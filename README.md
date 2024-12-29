# CSPC Lidar ROS2 Node and SDK

This software was obtained from an AliExpress seller.

AliExpress product link: https://www.aliexpress.com/item/1005005897673514.html

COIN-D4A DTOF 12M 360° scanning ranging LiDAR is suitable for mapping, navigation obstacle avoidance, environmental modeling

1. Install UDEV rules:
```
sudo cp sc_mini.rules /etc/udev/rules.d
```

2. Build
```bash
colcon build
```

3. Source workspace
```bash
source install/setup.bash
```

4. Launch demo
```
ros2 launch cspc_lidar demo.launch.py
```
