# Possible Commands
uint8 ADD_LINK=0
uint8 MOVE_LINK=1
uint8 MOVE_JOINT=2
uint8 REMOVE_LINK=3
uint8 REMOVE_JOINT=4
uint8 CHANGE_LINK_ORIGIN=5
uint8 CHANGE_JOINT_ORIGIN=6
uint8 CHANGE_LINK_COLLISION_ENABLED = 7
uint8 CHANGE_LINK_VISIBILITY=8
uint8 MODIFY_ALLOWED_COLLISIONS=9
uint8 REMOVE_ALLOWED_COLLISION_LINK=10
uint8 UPDATE_JOINT_STATE=11
uint8 ADD_SCENE_GRAPH=12
uint8 CHANGE_JOINT_POSITION_LIMITS=13
uint8 CHANGE_JOINT_VELOCITY_LIMITS=14
uint8 CHANGE_JOINT_ACCELERATION_LIMITS=15
uint8 ADD_KINEMATICS_INFORMATION=16
uint8 REPLACE_JOINT=17
uint8 CHANGE_COLLISION_MARGINS=18
uint8 ADD_CONTACT_MANAGERS_PLUGIN_INFO=19
uint8 SET_ACTIVE_DISCRETE_CONTACT_MANAGER=20
uint8 SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER=21
uint8 ADD_TRAJECTORY_LINK=22

# Specifies the type of command. Dictates which of the fields below are used
uint8 command

# ADD - Adds a link and the associated joint to the environment
tesseract_msgs/Link add_link
tesseract_msgs/Joint add_joint
bool add_replace_allowed

# MOVE_LINK - Changes the joint associated with the link
tesseract_msgs/Joint move_link_joint

# MOVE_JOINT - Changes the parent link associated with the joint
string move_joint_name
string move_joint_parent_link

# REMOVE_LINK - Removes the link from the environment
string remove_link

# REMOVE_JOINT - Removes the joint from the environment
string remove_joint

# REPLACE_JOINT - Replace the joint in the environment
tesseract_msgs/Joint replace_joint

# CHANGE_LINK_ORIGIN - Changes the origin of the link
string change_link_origin_name
geometry_msgs/Pose change_link_origin_pose

# CHANGE_JOINT_ORIGIN - Changes the origin of the joint
string change_joint_origin_name
geometry_msgs/Pose change_joint_origin_pose

# CHANGE_LINK_COLLISION_ENABLED - Enables/Disables collisions for the link
string change_link_collision_enabled_name
bool change_link_collision_enabled_value

# CHANGE_LINK_VISIBILITY - Enables/Disables visibility for the link
string change_link_visibility_name
bool change_link_visibility_value

# MODIFY_ALLOWED_COLLISIONS - Modify allowable collisions in the allowable collision matrix
uint8 modify_allowed_collisions_type # ADD, REMOVE, REPLACE
tesseract_msgs/AllowedCollisionEntry[] modify_allowed_collisions

# REMOVE_ALLOWED_COLLISION_LINK
string remove_allowed_collision_link

# UPDATE_JOINT_STATE
sensor_msgs/JointState joint_state

# ADD_SCENE_GRAPH Command
tesseract_msgs/SceneGraph scene_graph
tesseract_msgs/Joint scene_graph_joint
string scene_graph_prefix

# CHANGE_JOINT_POSITION_LIMITS Command
tesseract_msgs/StringLimitsPair[] change_joint_position_limits

# CHANGE_JOINT_VELOCITY_LIMITS Command
tesseract_msgs/StringDoublePair[] change_joint_velocity_limits

# CHANGE_JOINT_ACCELERATION_LIMITS Command
tesseract_msgs/StringDoublePair[] change_joint_acceleration_limits

# ADD_KINEMATICS_INFORMATION Command
tesseract_msgs/KinematicsInformation add_kinematics_information

# CHANGE_COLLISION_MARGINS Command
tesseract_msgs/CollisionMarginData collision_margin_data
tesseract_msgs/CollisionMarginOverrideType collision_margin_override_type

# ADD_CONTACT_MANAGERS_PLUGIN_INFO Command
tesseract_msgs/ContactManagersPluginInfo add_contact_managers_plugin_info

# SET_ACTIVE_DISCRETE_CONTACT_MANAGER Command
string set_active_discrete_contact_manager

# SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER Command
string set_active_continuous_contact_manager

# ADD_TRAJECTORY_LINK Command
string add_trajectory_link_name
string add_trajectory_link_parent_name
tesseract_msgs/JointTrajectory add_trajectory_link_traj
bool add_trajectory_link_replace_allowed
