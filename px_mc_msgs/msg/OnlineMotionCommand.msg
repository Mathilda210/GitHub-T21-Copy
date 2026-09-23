
# Trajectory type, including the following 2 types
uint8 type

# Joint-space trajectory
uint8 JOINT_TRAJECTORY=0

# Cartesian-space trajectory
uint8 CARTESIAN_TRAJECTORY=1

# Target arm index array; valid options are as follows
uint8[] target_arm_indices

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

# Array of trajectory points (each arm contains only one target position)
#
# Single target waypoint
# - Required field: position (joint-space or Cartesian-space)
# - Optional fields: velocity / acceleration
#
# Example (1000 Hz joint control):
#   target_point: {positions: [0.1,0.2,0.3], velocities: [0.01,0.01,0.01]}
TrajectoryPoint[] target_point

# Batch trajectory array (each arm contains multiple target positions)
#
# Batch trajectory
# - Each TrajectoryArray contains a set of trajectory points ordered by time.
#
# Example (30 Hz Cartesian-space trajectory):
#   batch_trajectory: [
#     {trajectory: [
#       {positions: [x1,y1,z1,qx1,qy1,qz1,qw1]},
#       {positions: [x2,y2,z2,qx2,qy2,qz2,qw2]}
#     ]}
#   ]
TrajectoryArray[] batch_trajectory

# Reference frame (interface reserved only; related functionality not yet implemented)
string reference_frame

# Assign a unique ID to the current motion command
uint32 command_id

# Whether the state machine needs to interpolate the target position
#
# When interpolation is enabled (need_interpolation=true), the state machine will:
#   1. Perform quintic polynomial interpolation between trajectory points
#   2. Generate a 1 kHz command stream
#   3. Automatically handle continuity of velocity and acceleration
bool need_interpolation

# Whether it is whole-body control (only valid for Cartesian-space commands)
bool is_whole_body

# Whether this is an emergency command (clears all commands in the current trajectory tracking buffer)
bool is_urgent

# Timestamp
builtin_interfaces/Time stamp

# Maximum allowed velocity (please refer to the joint maximum velocity limits in pr10_driver)
ArmConstraintParameters[] max_velocity

# Maximum allowed acceleration
ArmConstraintParameters[] max_acceleration

# Maximum allowed jerk
ArmConstraintParameters[] max_jerk

# Position deviation threshold for trajectory termination condition (recommended to leave unset and use system default)
float64 pos_tolerance

# Velocity deviation threshold for trajectory termination condition (recommended to leave unset and use system default)
float64 vel_tolerance
