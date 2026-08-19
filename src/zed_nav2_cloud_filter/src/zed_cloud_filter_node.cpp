#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class ZedCloudFilterNode : public rclcpp::Node
{
public:
  ZedCloudFilterNode()
  : Node("zed_cloud_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_  = declare_parameter<std::string>(
      "input_cloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/zed/filtered_cloud");
    target_frame_       = declare_parameter<std::string>("target_frame", "base_link");

    min_range_ = declare_parameter<double>("min_range", 0.25);
    max_range_ = declare_parameter<double>("max_range", 6.0);

    // floor/sky removal in target_frame
    min_z_ = declare_parameter<double>("min_z", 0.05);
    max_z_ = declare_parameter<double>("max_z", 1.50);

    // keep 1 of every N points (cheap downsample)
    decimate_n_ = declare_parameter<int>("decimate_n", 8);
    if (decimate_n_ < 1) decimate_n_ = 1;

    // publish at most X Hz (0 = no throttle)
    throttle_hz_ = declare_parameter<double>("throttle_hz", 10.0);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ZedCloudFilterNode::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "input:        %s", input_cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output:       %s", output_cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_frame: %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), "TF lookup mode: latest available transform (TimePointZero)");
    RCLCPP_INFO(
      get_logger(), "filter: range %.2f..%.2f m, z %.2f..%.2f m, decimate %d, %.1f Hz",
      min_range_, max_range_, min_z_, max_z_, decimate_n_, throttle_hz_);
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Throttle
    if (throttle_hz_ > 0.0) {
      auto now = this->now();
      if (last_pub_time_.nanoseconds() != 0) {
        double dt = (now - last_pub_time_).seconds();
        if (dt < (1.0 / throttle_hz_)) return;
      }
      last_pub_time_ = now;
    }

    // TF transform into target_frame.
    // The camera→base_link transform is a rigid (static) mount, so the exact cloud
    // timestamp is irrelevant. Looking up at the cloud stamp fails with "extrapolation
    // into the future" whenever the cloud is stamped slightly ahead of the latest TF
    // sample (e.g. when depth_stabilization force-enables positional tracking). Those
    // drops starve /zed/filtered_cloud and trip the obstacle node's cloud-timeout
    // fail-safe. Use the latest available transform (TimePointZero) instead.
    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
      auto tf = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, cloud_tf, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF failed (%s -> %s): %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    // Output cloud with xyz only
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg->header.stamp;   // keep the camera capture time, not the TF time
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.is_bigendian = cloud_tf.is_bigendian;
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    std::vector<float> pts;
    pts.reserve(static_cast<size_t>(cloud_tf.width) * 3 / static_cast<size_t>(decimate_n_));

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_tf, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_tf, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_tf, "z");

    int idx = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++idx) {
      if (decimate_n_ > 1 && (idx % decimate_n_) != 0) continue;

      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      const double r = std::sqrt((double)x * (double)x + (double)y * (double)y);
      if (r < min_range_ || r > max_range_) continue;
      if (z < min_z_ || z > max_z_) continue;

      pts.push_back(x);
      pts.push_back(y);
      pts.push_back(z);
    }

    const size_t n_points = pts.size() / 3;
    mod.resize(n_points);
    out.width = static_cast<uint32_t>(n_points);

    sensor_msgs::PointCloud2Iterator<float> o_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> o_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> o_z(out, "z");

    for (size_t i = 0; i < n_points; ++i, ++o_x, ++o_y, ++o_z) {
      *o_x = pts[i * 3 + 0];
      *o_y = pts[i * 3 + 1];
      *o_z = pts[i * 3 + 2];
    }

    pub_->publish(out);
  }

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string target_frame_;
  double min_range_, max_range_;
  double min_z_, max_z_;
  double throttle_hz_;
  int decimate_n_;

  rclcpp::Time last_pub_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
