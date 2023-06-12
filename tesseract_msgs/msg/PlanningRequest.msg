tesseract_msgs/EnvironmentState environment_state
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to planning

string name # The name of the taskflow to use
string executor # The executor to leverage, if empty default one is used.
string instructions # This should an xml string of the command language instructions
bool dotgraph # Enable if DOT Graph should be generated and returned
bool debug # Enable debug content
bool save_io # Enable saving of input and output for task infos

tesseract_msgs/PlannerProfileRemapping move_profile_remapping
tesseract_msgs/PlannerProfileRemapping composite_profile_remapping
