# SDO command message for controlling the robot system, including hand homing.
# It contains a timestamp, a command identifier, a command type, and optional target joint names.

# Timestamp
builtin_interfaces/Time stamp 

# Command ID 
uint32 command_id   

# Command type; use 4 for homing
uint8 command   

# Target joint names; use [] for homing
string[] joint_names             

 
# Command enumeration

# Dexterous hand homing
uint8 RESET_HAND_TO_ZERO_POS=4    