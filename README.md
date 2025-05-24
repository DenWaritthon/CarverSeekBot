# CarverSeekBot ROS2
Project Restore CarverCap to CarverSeekBot

Update 17/05/68

Checklit 
## Checklist
- [x] Modbus
- [x] Lidar
- [x] Controller from AJ.Nook 

> Controller need to find USB


# H2U Test

1. Lidar test

```bash
ros2 launch robot_bringup lidar_test.launch.py
```

2. Motor Controller

```bash
ros2 launch robot_controller diff_drive_controller.py
```