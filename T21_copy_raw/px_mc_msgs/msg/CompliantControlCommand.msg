std_msgs/Header header       
# Message header
 
uint8[] target_arm_indices   
# Target hand indices
# 4: Left hand
# 5: Right hand
 
string controller_type       
# Controller type
# For admittance control, always set to ADMITTANCE

bool enable                  
# Admittance control switch
# true: Enabled
# false: Disabled
 
uint8 output_mode            
# Output mode
# 0: Position output
# 1: Torque output
# For admittance control, always set to 0