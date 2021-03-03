string id # Environment Name
uint64 revision # This is the revision number so others know when it changes.
tesseract_msgs/Environment environment # Used if id not found

# This message contains information about the state, i.e. the positions of its joints and links
sensor_msgs/JointState joint_state

# Joints that may have multiple DOF are specified here
sensor_msgs/MultiDOFJointState multi_dof_joint_state
