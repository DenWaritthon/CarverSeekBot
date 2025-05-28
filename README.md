# CarverSeekBot ROS2
Project Restore CarverCap to CarverSeekBot

Update 28/05/68

Checklit 
## Checklist
- [x] Modbus
- [x] Lidar
- [x] Controller from AJ.Nook 
- [x] Nav2 Done

**Need to reformat code**

# H2 Launch this project
1. Launch bringup
```bash
ros2 launch robot_bringup robot_bringup.launch.py
```
2. Launch navigation (Nav2)
```bash
ros2 launch robot_navigation navigation.launch.py
```

# H2U Test

1. Lidar test

```bash
ros2 launch robot_bringup lidar_test.launch.py
```

2. Motor Controller

```bash
ros2 launch robot_controller diff_drive_controller.py
```

3. IMU Micro ROS
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/m5c
```