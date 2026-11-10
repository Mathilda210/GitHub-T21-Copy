# Structured coordinate transform array; contents depend on the robot model
geometry_msgs/TransformStamped[] transforms

# Published content (common to all robot models)
w -> left_arm_link_6
waist_link_4 -> right_arm_link_6     (waist -> right arm end)
waist_link_4 -> head_link_0          (waist -> head)
base_link_0 -> left_arm_link_6       (base -> left arm end)
base_link_0 -> right_arm_link_6      (base -> right arm end)
base_link_0 -> waist_link_4          (base -> waist head)
base_link_0 -> waist_link_3          (base -> waist camera)
waist_link_0 -> left_arm_link_6      (waist root -> left arm end)
waist_link_0 -> right_arm_link_6     (waist root -> right arm end)

# Published content (additional entries for the Fold robot model; publishes dynamic tf after the folded-leg motor motion)
base_link_0 -> waist_link_0         (dynamic compensation: base root -> waist root)
base_odom_link -> waist_link_0      
base_odom_link -> waist_link_3      (dynamic compensation: base odom -> waist camera)
base_odom_link -> head_link_0       (dynamic compensation: base odom -> head)
 