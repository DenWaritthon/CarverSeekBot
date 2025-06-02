# **M5StickC IMU**

The M5StickC functions as the robot's Inertial Measurement Unit (IMU). Its primary role is to directly transmit raw `gyroscope` and `accelerometer` data from its internal sensors to the Micro-ROS Agent.

All subsequent processing, including sensor calibration and axis transformations, is performed on the robot's main control unit (NUC).

> [!NOTE]
> When configuring the M5StickC, please verify the port used on the NUC. The current configuration utilizes the `/dev/m5c` port.