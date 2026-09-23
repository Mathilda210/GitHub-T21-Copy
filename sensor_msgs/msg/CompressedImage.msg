// WARNING: plusieurs interrogations sur cette interface 
// Elle est en c++ alors que les autres sont en python 
// Definition etrange j'ai pas tout compris 



// Message header (timestamp and reference frame)
// header de format std_msgs::msg::Header (qui vient de ROS2)
std_msgs::msg::Header header;

builtin_interfaces::msg::Time stamp;   // Timestamp
std::string frame_id;                  // Coordinate frame name

// Compression format
std::string format;                    // Encoding format, e.g. "h264;1280x720"

// Compressed video data
std::vector<uint8_t> data;             // H.264 compressed video stream buffer