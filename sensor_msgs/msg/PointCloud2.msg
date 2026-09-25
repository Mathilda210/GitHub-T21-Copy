# ROS 2 sensor_msgs/PointCloud2 message definition.

# Message header containing the timestamp and reference frame ID.
std_msgs/Header header

# header.stamp: Timestamp of the point cloud message.
# header.frame_id: Coordinate frame in which the point cloud is expressed.

# Point cloud dimensions and organization.
# If height is 1, the point cloud is unordered and width is the total number
# of points.
uint32 height              # Height of the point cloud in points.
uint32 width               # Width of the point cloud in points.

# Definitions of the fields contained in each point, such as x, y, z,
# intensity, or rgb.
# Each PointField contains a name, byte offset, data type, and element count.
sensor_msgs/PointField[] fields

# Whether the point data uses big-endian byte order.
bool is_bigendian

# Number of bytes occupied by a single point across all fields.
uint32 point_step

# Number of bytes in a row of points. For organized clouds, this is usually
# width * point_step.
uint32 row_step

# Raw binary point cloud data, interpreted according to fields.
uint8[] data

# Whether the point cloud contains only valid points.
# If false, invalid points such as NaN or Inf values may be present.
bool is_dense
