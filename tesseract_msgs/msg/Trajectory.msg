# Header
std_msgs/Header header

# The namespace the trajectory is associated with
# This is mostly used by visualization tools
string ns

# A description of the trajectory
string description

# (Optional) Override the existing Tesseract Environment
Environment environment

# (Optional) Additional Commands to be applied to environment prior to trajectory visualization
tesseract_msgs/EnvironmentCommand[] commands

# (Required) Initial Environment Joint State
tesseract_msgs/StringDoublePair[] initial_state

############################################
# One of the following options is required #
############################################

# Tesseract Command Language Instructions
string instructions

# If instructions are provided this will be ignored
tesseract_msgs/JointTrajectory[] joint_trajectories
