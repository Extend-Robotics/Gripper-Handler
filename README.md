# Gripper-Handler
ROS package that interfaces with the rh_p12_rn_a package and allows easy to use control through two topics:

Position control uses /gripper_position , uses a Float64 message with 0 for open, 1.0 for closed

Force control uses /gripper_force , uses a Float64 message with 0.5 default current (recommended value, it is set through /gripper_handler/default_force parameter)

