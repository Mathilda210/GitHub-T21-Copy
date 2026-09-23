# Message header containing the timestamp and reference frame.
std_msgs/Header header

# Image dimensions in pixels.
uint32 height
uint32 width

# Camera distortion model and parameters.
# Supported models include plumb_bob (5 parameters) and
# rational_polynomial (8 parameters).
string distortion_model

# Distortion coefficients.
float64[] D

# Camera intrinsic matrix, stored in row-major order (3x3).
float64[9] K

# Rectification matrix (3x3).
float64[9] R

# Projection matrix (3x4).
float64[12] P

# Stereo camera binning factors.
uint32 binning_x
uint32 binning_y

# Region of interest (ROI) parameters.
sensor_msgs/RegionOfInterest roi