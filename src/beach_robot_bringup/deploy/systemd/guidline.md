## ติดตั้ง:
```bash
sudo cp ~/beach_robot_ws/src/beach_robot_bringup/deploy/systemd/beach_robot_bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable beach_robot_bringup.service
sudo systemctl start beach_robot_bringup.service
sudo systemctl status beach_robot_bringup.service
```