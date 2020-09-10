tesseract_msgs/TesseractState tesseract_state

string name # The name of the planner to use
string instructions # This should an xml string of the command language instructions
string seed # This should an xml string of the command language instructions (Optional)
uint16 num_threads # This is the number of thread to use during planning

tesseract_msgs/PlannerProfileRemapping plan_profile_remapping
tesseract_msgs/PlannerProfileRemapping composite_profile_remapping
