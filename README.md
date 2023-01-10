# ixblue_quadrans_ros2
This is a ROS2 node for IMU_RAW_DATA format for iXblue Quadrans. It contains a driver to interpret the binary data output and it proceeds to the mechanization in a ROS2 node publishing a sensor_msgs::msg::Imu (attitude and raw data) at 200 Hz. Covariance matrices output is disabled in order to allow computation at 200 Hz, however raw data are available to compute them at a lower frequency in another node.

Setup:
Change the Quadrans output to IMU_RAW_DATA over UDP @ 200Hz.

Start the node:
ros2 run quadrans imu_pub

Result:
A sensor_msgs::msg::Imu message containing delta angles, delta velocities and an attitude quaternion.
Access the attitude quaternion to extract your attitude angles with your own convention.
