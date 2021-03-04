# Header
std_msgs/Header header

# Tesseract State
EnvironmentState environment_state
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to trajectory visualization

# Tesseract Command Language Instructions
string instructions

# If instructions are provided this will be ignored
tesseract_msgs/JointState[] joint_trajectory
