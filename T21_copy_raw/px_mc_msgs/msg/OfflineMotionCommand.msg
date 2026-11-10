# Trajectory type, including the following 3 types
uint8 type

# Joint-space trajectory
uint8 JOINT_TRAJECTORY=0

# Cartesian-space trajectory
uint8 CARTESIAN_TRAJECTORY=1

# Torque trajectory (reserved interface; functionality not yet implemented)
uint8 TORQUE_COMMAND=2

# Target arm index array
uint8[] target_arm_indices

# Arm index array that needs to move synchronously
uint8[] sync_arm_indices

# Left arm
uint8 LEFT_ARM=0

# Right arm
uint8 RIGHT_ARM=1

# Waist
uint8 WAIST=2

# Head
uint8 HEAD=3

# Left hand
uint8 LEFT_HAND=4

# Right hand
uint8 RIGHT_HAND=5

# Asynchronous trajectory point array (each arm moves independently)
#
# TrajectoryPoint structure:
#   positions (double[]): target position array
#   velocities (double[]): target velocity array
#   accelerations (double[]): target acceleration array
#   time_from_start (double): time offset from the start of the trajectory (seconds)
#   (reserved interface; functionality not yet implemented)
TrajectoryPoint[] async_trajectory

# Synchronous trajectory point array (multi-arm coordinated motion)
TrajectoryPoint[] sync_trajectory

# Reference frame (reserved interface; functionality not yet implemented)
string reference_frame

# Assign an ID to this motion command
uint32 command_id

# Whether this is an emergency command (clears all commands in the current trajectory tracking buffer)
bool is_urgent

# Timestamp
builtin_interfaces/Time stamp