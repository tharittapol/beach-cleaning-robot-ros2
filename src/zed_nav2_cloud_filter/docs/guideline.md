# เช็ค NVIDIA driver / CUDA ก่อน
```bash
nvidia-smi
```

## ติดตั้ง dependency ****skip this
```bash
sudo apt update
sudo apt install -y zstd
sudo apt install -y python3-serial python3-yaml
```

# ติดตั้ง CUDA 13.0 จาก NVIDIA repo ****skip this
## เพิ่ม keyring/repo (สำหรับ Ubuntu 22.04)
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
```
## ติดตั้ง CUDA 13.0 toolkit: ****skip this
```bash
sudo apt install -y cuda-toolkit-13-0
```
## ตั้ง PATH/LD_LIBRARY_PATH ให้ใช้ CUDA 13.0 เป็นค่าเริ่มต้น ****skip this
ใส่ใน ~/.bashrc:
```bash
echo 'export CUDA_HOME=/usr/local/cuda-13.0' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```
แล้วเช็ค:
```bash
nvcc --version
```
## ทำให้ /usr/local/cuda ชี้ไป CUDA 13.0 ****skip this
```bash
sudo ln -sfn /usr/local/cuda-13.0 /usr/local/cuda
```
เช็ค:
```bash
ls -l /usr/local/cuda
```

## ดาวน์โหลดและติดตั้ง ZED SDK
ไปที่หน้า ZED SDK for Linux แล้วดาวน์โหลด .run ให้ตรงกับ Ubuntu/CUDA ของเครื่อง
`install => "ZED SDK for JetPack 6.1 and 6.2 (L4T 36.4) 5.2 (Jetson Orin, CUDA 12.6)"`
จากนั้นติดตั้ง:
```bash
cd ~/Downloads
chmod +x ZED_SDK_Ubuntu*.run
./ZED_SDK_Ubuntu*.run
```

## ตั้ง env ให้ CMake หา ZED SDK ได้แน่นอน ****skip this
```bash
echo 'export ZED_DIR=/usr/local/zed' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/zed' >> ~/.bashrc
source ~/.bashrc
```

## อัปเดต zed-ros2-wrapper ให้เป็น v5.2.0 ****skip this
```bash
cd ~/beach_robot_ws/src/zed-ros2-wrapper
```
```bash
git status
git describe --tags --always
```
```bash
git fetch --all --tags
git checkout v5.2.0
```
```bash
git stash -u
git checkout v5.2.0
```

## Build workspace
```bash
cd ~/beach_robot_ws
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --executor sequential
source install/setup.bash
```

## เช็คว่ากล้องเห็นในระบบ
```bash
lsusb | grep -i stereolabs || true
```
## ทดสอบเปิดกล้องจริง (ต้องผ่าน)
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zed
```
```bash
ros2 topic list | grep zed
ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --once
```