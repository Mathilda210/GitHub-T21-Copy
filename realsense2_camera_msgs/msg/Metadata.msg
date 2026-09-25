# Message header containing the timestamp and reference frame.
std_msgs/Header header

# JSON-formatted metadata string containing camera parameters and status.
string json_data

# Common fields in the RealSense camera metadata JSON string:

# Timing
# - frame_timestamp: Frame timestamp.
# - frame_counter: Frame counter.
# - system_time: System time.

# Exposure
# - exposure_time: Exposure time in microseconds.
# - auto_exposure: Automatic exposure enabled flag.

# Gain
# - gain: Gain value.
# - auto_gain: Automatic gain enabled flag.

# White balance
# - white_balance: White balance value.
# - auto_white_balance: Automatic white balance enabled flag.

# Depth
# - depth_units: Depth units, expressed in meters per pixel value.
# - depth_scale: Depth scale factor.

# Camera intrinsics
# - intrinsic: Camera intrinsic matrix.
# - ppx, ppy: Principal point coordinates.
# - fx, fy: Focal lengths.

# IMU data (for cameras such as the D435i)
# - accel_data: Accelerometer data.
# - gyro_data: Gyroscope data.
# - accel_timestamp: Accelerometer timestamp.
# - gyro_timestamp: Gyroscope timestamp.

# Other parameters
# - brightness: Brightness.
# - contrast: Contrast.
# - saturation: Saturation.
# - sharpness: Sharpness.
# - laser_power: Laser power.
# - emitter_enabled: Emitter enabled flag.
# - visual_preset: Visual preset.
