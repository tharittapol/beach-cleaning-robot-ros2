# beach_robot_coverage_nav2
แพ็กเกจนี้ทำระบบ “ทำความสะอาดแบบครอบคลุมพื้นที่” (coverage) สำหรับหุ่นวิ่งบนชายหาด โดยใช้:
- Same-direction coverage: ทุกแถววิ่งจาก `x=0` ไป `x=5`
- Return loop นอกพื้นที่ด้วยรัศมีเลี้ยวไม่น้อยกว่า `2.1 m`
- Spiral coverage (ก้นหอยจากวงนอกเข้าวงใน)
- ใช้ Nav2 เป็นตัว “นำทางไปตาม waypoint” และทำ obstacle avoidance
- ใช้ Keepout / Boundary mask บังคับให้หุ่นไม่ออกนอกเขต (mapless ในความหมายว่าไม่ต้อง SLAM map)

## Nodes ที่แพ็กเกจนี้มี
1) coverage_follow_waypoints
    - สร้าง waypoint แบบ boustrophedon หรือ spiral แล้วส่งเข้า Nav2 ผ่าน Action FollowWaypoints
    - publish preview path ที่ `/coverage/path` สำหรับดูใน RViz

2) generate_keepout_mask
    - สร้างไฟล์ keepout_mask.pgm + keepout_mask.yaml สำหรับ boundary/keepout (นอกพื้นที่เป็น “ห้ามเข้า”)

## Launch ที่แพ็กเกจนี้มี
1) beach_cleaning_bringup.launch.py
    - รันครบ: keepout mask server + costmap filter info server + nav2 + coverage planner

## Flow เริ่มต้น
- พื้นที่ `5.0 x 3.6 m`
- แถวอยู่ที่ `y = 0.3, 0.9, 1.5, 2.1, 2.7, 3.3`
- ทุกแถวเริ่ม `(x=0, y=lane)` และจบ `(x=5, y=lane)`
- ค่า spawn เริ่มต้นคือ `(x=0.0, y=0.3, yaw=0.0)` ตรงกับต้นแถวแรก
- หลังแถว 1–3 วนกลับด้านบน
- หลังแถว 4–5 วนกลับด้านล่าง
- เส้นตรงหลักของทุก return loop ขนานกับแนว lane
- แต่ละ loop คำนวณระดับ Y ใกล้พื้นที่ที่สุดแยกกัน
- ทุกส่วนโค้งใช้รัศมีอย่างน้อย `2.1 m`
- หลังแถว 6 จบงานที่ `(5.0, 3.3)`

## กำหนดพิกัดพื้นที่แบบ “มุมล่างซ้ายเป็น origin”
กำหนด origin_x/origin_y ให้เป็นมุมล่างซ้ายใน map
1) รัน localization ให้ได้ map.
2) ขับหุ่นไปที่ “มุมล่างซ้ายจริง”
3) อ่านค่า (x,y) ของหุ่นใน map ด้วย:
    ```bash
    ros2 run tf2_ros tf2_echo map base_link
    ```
    จด Translation: x=..., y=...
4) เอาค่านี้ไปใส่เป็น:
    - area.origin_x = x
    - area.origin_y = y

## ค่า Coverage เริ่มต้น
- area.width = 5.0
- area.height = 3.6
- tool_width = 0.60
- lane_spacing = 0.60
- lane_center_offset = 0.30
- coverage_path_mode = same_direction_loops
- upper_return_count = 3
- boundary_margin = 0.0 เพื่อให้แนววิ่งเริ่มที่ x=0 และจบ x=5
- waypoint_step = 0.50
- turn_style = arc
- turn_radius = 2.10
- minimum_turn_radius = 2.10
- num_passes = 1

return loop ออกนอกพื้นที่ จึงต้องใช้ `use_keepout:=false` หรือทำ keepout mask
ให้ครอบคลุมพื้นที่วนกลับด้วย

เรื่อง “หุ่นหันหน้าขนานด้านยาวตอนเริ่ม”
planner จะสร้างแถวแรกให้ yaw = แนวแกน X ของสี่เหลี่ยม
ดังนั้นถ้า area.yaw ทำให้แกน X ขนานชายหาด หุ่นจะเริ่มหันถูกทิศ

## area.yaw คืออะไร (ทำให้ขนานชายหาด)
- area.yaw คือมุมหมุนของสี่เหลี่ยม (เรเดียน) ใน frame map
- ถ้าแกน X ของ map ขนานชายหาดอยู่แล้ว → area.yaw = 0.0
- ถ้า map เอียงจากแนวชายหาด → ตั้ง area.yaw ให้แกน X ของสี่เหลี่ยม “ขนานชายหาด”

แปลงองศาเป็นเรเดียน:
10° = 0.1745
15° = 0.2618

## Keepout / Boundary Mask
### Example Generate mask ให้เป็น 30x10
```bash
ros2 run beach_robot_coverage_nav2 generate_keepout_mask --ros-args \
  -p output_dir:=$HOME/beach_robot_ws/src/beach_robot_coverage_nav2/config \
  -p mask_basename:=keepout_mask \
  -p area.origin_x:=0.0 -p area.origin_y:=0.0 \
  -p area.width:=30.0 -p area.height:=10.0 -p area.yaw:=0.0 \
  -p resolution:=0.10 -p outer_margin:=5.0
```
หมายเหตุ: mask map ไม่ควรหมุนด้วย yaw ใน YAML origin (ควรเป็น 0.0)
การหมุนพื้นที่ให้ขนานชายหาดให้ใช้ area.yaw ใน planner แทน
### เช็คว่า keepout ทำงานจริง
หลังรัน bringup:
```bash
ros2 topic echo /keepout_mask --once
ros2 topic echo /costmap_filter_info --once
```

## Run step (ระบบเต็ม)
### Terminal 1: localization
```bash
ros2 launch beach_robot_localization localization.launch.py
```
เช็ค TF:
```bash
ros2 run tf2_ros tf2_echo map base_link
```
### Terminal 2: Nav2 + keepout + coverage
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py
```

รันเส้นทางค่าเริ่มต้น:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  use_keepout:=false
```

bringup จะตั้ง static `map->odom` ให้หุ่นเริ่มที่ `(0.0, 0.3)` หันหน้า `+X`
ตรงกับ waypoint แรก จึงเริ่มงานด้วยการเดินตรงทันที ตรวจตำแหน่งก่อนเริ่มได้ด้วย:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

หากใช้ GNSS/global localization ที่ publish `map->odom` เอง ให้ตั้ง
`publish_map_to_odom_tf:=false` และนำหุ่นไปวางที่ต้นแถวแรกจริงแทน

หรือก้นหอยวงนอกเข้าวงใน:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  coverage_pattern:=spiral \
  lane_spacing:=0.60 \
  turn_radius:=0.30
```

ปรับมุมพื้นที่ให้ตรงกับแนววิ่งจริง:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  use_keepout:=false \
  area_origin_x:=0.0 area_origin_y:=0.0 area_yaw:=0.0
```

## ปรับพารามิเตอร์ใน launch
ไปแก้ใน launch/beach_cleaning_bringup.launch.py ที่ node coverage_follow_waypoints ให้มี:
- area.width: 5.0
- area.height: 3.6
- origin_x/origin_y: มุมล่างซ้ายใน map (ให้อ่าน x,y จาก tf2_echo แล้วใส่)
- area.yaw: ทำให้วิ่งขนานชายหาด

## Visualization / Debug
### RViz2
Add displays:
- TF
- Map → /keepout_mask
- Costmap → /global_costmap/costmap, /local_costmap/costmap
- Odometry → /odometry/local และ /odometry/global
- Path (ถ้ามี)
### เช็ค Nav2 สั่งงานจริงไหม
```bash
ros2 topic echo /cmd_vel
```

## ZED obstacle safety stop

เมื่อ point cloud ตรวจพบวัตถุด้านหน้าภายใน 2.0 m จากกล้อง ZED ระบบจะ:

- publish `/safety/e_stop=true` ให้ ESP32 bridge บังคับล้อหยุด
- ร้อง buzzer เป็นระยะ
- คง Nav2 goal เดิมไว้
- เมื่อทางโล่งต่อเนื่องเกิน 3 วินาที publish `/safety/e_stop=false` แล้ว Nav2 วิ่งต่อ
- ถ้า point cloud timeout ระบบจะแจ้งเตือนใน log แต่ไม่ trigger E-stop

Coverage bringup เปิด **กล้อง ZED + สาย E-stop** (bridge OR `/e_stop` จอย กับ `/safety/e_stop`)
ให้พร้อม แต่ **ไม่ได้ spawn ตัว detector เอง** — รัน node ตรวจจับแยก (persistent เหมือน GNSS):

```bash
# เทอร์มินอล 1: bringup (เปิดกล้อง ZED ให้แล้ว)
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py
# เทอร์มินอล 2: detector แยก — กล้องเปิดจาก bringup แล้ว จึง launch_zed:=false
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py launch_zed:=false
```

ทดสอบกล้อง + safety node แบบ standalone (detector launch เปิดกล้องเองในตัว):

```bash
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py
```

โปรไฟล์ `zedm_orin_nano_depth.yaml` ปิด ZED positional tracking และ
`depth_stabilization` แล้ว แต่ยังเปิด depth point cloud สำหรับ obstacle detection

สำหรับหยุดล้อจริง ต้องมี `beach_robot_esp32_bridge` รันอยู่ด้วย

ถ้า ZED camera และ `/zed/filtered_cloud` รันอยู่แล้ว:

```bash
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py launch_zed:=false
```

เมื่อ `launch_zed:=false` cloud ที่ส่งเข้า `cloud_topic` ต้องอยู่ใน frame `base_link`
เท่านั้น หาก frame ไม่ตรง detector จะไม่ใช้เฟรมนั้นและจะแจ้ง warning เพื่อป้องกันการ
ตีความแกนกล้องเป็นแกนหุ่นยนต์

ดูสถานะ:

```bash
ros2 topic echo /safety/obstacle_stop
ros2 topic echo /safety/obstacle_distance
ros2 topic echo /safety/obstacle_point_count
ros2 topic echo /safety/obstacle_bounds
ros2 topic echo /safety/e_stop
```

`min_forward_distance`, `stop_distance`, `/safety/obstacle_distance` และระยะใน log
อ้างอิงจากตำแหน่งกล้อง โดย node ใช้ `camera_forward_offset=0.71245 m` แปลงขอบเขต
เข้า frame `base_link` ก่อนตรวจ point cloud ดังนั้น `stop_distance=2.0` จะตรวจถึง
X=2.71245 m ใน `base_link`

`/safety/obstacle_point_count=-1` หมายถึง cloud timeout และไม่มีข้อมูลเฟรมใหม่
ไม่ใช่จำนวน obstacle point จริง

สำหรับตรวจ false positive ให้เพิ่ม PointCloud2 `/safety/obstacle_points` ใน RViz2
topic นี้แสดงเฉพาะจุดที่ detector นับว่าอยู่ภายในกล่องตรวจจับ
ถ้า PointCloud2 แสดงผลไม่ได้ ให้เพิ่ม MarkerArray `/safety/obstacle_markers` แทน:
กล่องตรวจจับเป็นสีเขียวโปร่ง และจุดที่ถูกนับเป็นสีแดง

ขอบเขตตรวจจับเป็นกล่องที่กำหนดระยะจากกล้อง แล้วแปลงไปตรวจใน frame `base_link`:

- ลึก/ด้านหน้า: `min_forward_distance` ถึง `stop_distance` วัดจากกล้อง
- กว้าง Y: `box_width` เป็นความกว้างรวมคงที่ตลอดแนว X จึงเป็นกล่องตรง ไม่บานออกแบบ cone
- สูง Z: `min_z` ถึง `max_z`
- ต้องมีอย่างน้อย `min_points` จุดในกล่องจึงถือว่าเป็น obstacle; ค่า default คือ 1000 จุด

ตัวอย่างกล่องตรวจจับลึก 0.30-2.00 m, กว้างรวม 0.80 m, สูง 0.15-1.40 m:

```bash
  ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py \
  min_forward_distance:=0.30 stop_distance:=2.0 \
  box_width:=0.80 min_z:=0.15 max_z:=1.40 \
  min_points:=1000 clear_time_sec:=3.0
```

`filter_min_range`, `filter_max_range`, `filter_min_z`, `filter_max_z` ใช้ปรับ
ขอบเขต point cloud ชั้นแรก ค่าชั้น filter ต้องครอบคลุมกล่อง detector เสมอ เช่น
ถ้าต้องตรวจถึงความสูง 1.80 m ให้เพิ่มทั้ง `max_z:=1.80 filter_max_z:=1.80`
ค่า `filter_*` จาก launch นี้มีผลเมื่อ `launch_zed:=true` เท่านั้น
เมื่อ `launch_zed:=true` ค่า `cloud_topic` จะถูกส่งต่อเป็น output topic ของ cloud
filter ด้วย จึงเปลี่ยน topic ได้โดย detector และ filter ยังคงเชื่อมต่อกัน

ค่า `use_static_tf:=true` ของ launch นี้ publish เฉพาะ static TF ของ ZED
(`base_link` ไป `zed_camera_link` และ `zed_imu_link`) ไม่ได้เปิด TF ของ sensor อื่น

เช็กค่าที่ node ใช้งานหลัง launch:

```bash
ros2 param dump /zed_obstacle_stop
ros2 param dump /zed_cloud_filter_node
```

### ดู graph
```bash
rqt_graph
```
### Debug “ทิศผิด / วิ่งสลับแกน”
อาการ: หุ่นวิ่งเป็นแถวสั้น (10 m) แทน 30 m
แก้: สลับ area.width/area.height หรือ area.yaw ผิด
