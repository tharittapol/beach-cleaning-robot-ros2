## ต้องรู้ VID/PID ของอุปกรณ์
```bash
udevadm monitor --udev --property
```

## ติดตั้ง rule:
```bash
sudo cp ~/beach_robot_ws/src/beach_robot_bringup/deploy/udev/99-beach-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## เช็ค:
```bash
ls -l /dev/ttyESP32 /dev/input/js_joy /dev/ttyGNSS /dev/cam
```

## เช็ค Logitech F710 เมื่อเสียบ ZED แล้ว joy ไม่ส่งข้อมูล

ถ้า `joy_node` เปิด `/dev/input/js_joy` ได้ แต่ `ros2 topic echo /joy` หรือ
`jstest` ไม่มีค่า axis/button ขยับ ปัญหามักอยู่ที่ receiver/USB radio ไม่ใช่ ROS.
F710 ใช้ 2.4 GHz และมักถูกรบกวนจากสาย/อุปกรณ์ USB3 เช่น ZED Mini ได้ง่ายมาก.

แนะนำให้เสียบ receiver ผ่านสาย USB extension สั้น ๆ แล้ววางให้ห่างจาก ZED/สาย USB3
อย่างน้อย 30-50 cm. ถ้าเป็นไปได้ให้แยก receiver ไปอยู่คนละด้านกับสาย ZED.

คำสั่งเช็คสด:

```bash
ls -l /dev/input/js_joy
udevadm info --query=property --name=/dev/input/js_joy | grep -E 'ID_PATH|ID_VENDOR|ID_MODEL'
lsusb -t
jstest --event /dev/input/js_joy
ros2 topic echo /joy
```

ถ้า `jstest` ขยับได้ แต่ `/joy` ไม่ขยับ ให้ restart `joy_node`.
ถ้า `jstest` ไม่ขยับด้วย ให้แก้ที่ receiver/ระยะ/ถ่าน/โหมด X-D ก่อน.
