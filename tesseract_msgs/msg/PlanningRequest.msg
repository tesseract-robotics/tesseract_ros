tesseract_msgs/EnvironmentState environment_state
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to planning

string name # The name of the taskflow to use
string executor # The executor to leverage, if empty default one is used.
string input # This should an xml string of tesseract_common::AnyPoly
bool dotgraph # Enable if DOT Graph should be generated and returned
bool debug # Enable debug content
bool save_io # Enable saving of input and output for task infos
