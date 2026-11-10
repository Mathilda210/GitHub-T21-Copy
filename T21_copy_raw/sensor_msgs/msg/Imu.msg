# ROS 2 sensor_msgs/Imu message definition.

# Message header containing the timestamp and reference frame ID.
std_msgs/Header header

# header.stamp: Timestamp of the IMU message.
# header.frame_id: Coordinate frame in which the IMU data is expressed.

# Optional orientation estimate.
# Set the first element of the covariance array to -1 if no valid estimate is
# available.
geometry_msgs/Quaternion orientation           # Orientation quaternion (x, y, z, w).
float64[9] orientation_covariance               # 3x3 row-major orientation covariance matrix.

# Angular velocity in radians per second.
geometry_msgs/Vector3 angular_velocity          # Angular velocity around the X, Y, and Z axes.
# x: Roll-axis angular velocity.
# y: Pitch-axis angular velocity.
# z: Yaw-axis angular velocity.
float64[9] angular_velocity_covariance          # 3x3 row-major angular velocity covariance matrix.

# Linear acceleration in meters per second squared.
geometry_msgs/Vector3 linear_acceleration       # Acceleration along the X, Y, and Z axes.
# x: Acceleration along the X axis.
# y: Acceleration along the Y axis.
# z: Acceleration along the Z axis, usually the direction of gravity.
float64[9] linear_acceleration_covariance       # 3x3 row-major linear acceleration covariance matrix.
 