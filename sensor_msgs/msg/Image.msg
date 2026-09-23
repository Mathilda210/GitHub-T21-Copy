# Message header containing the timestamp and reference frame.
std_msgs/Header header

# Image dimensions in pixels.
uint32 height
uint32 width

# Pixel encoding, for example: rgba8, bgr8, mono8, 16UC1, or yuv422_yuy2.
string encoding

# Whether the image data uses big-endian byte order.
uint8 is_bigendian

# Number of bytes per image row (row stride).
uint32 step

# Pixel data. Its length is step * height.
uint8[] data