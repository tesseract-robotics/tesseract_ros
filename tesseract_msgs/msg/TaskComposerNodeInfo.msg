# This message corresponds to tesseract_planning::TaskComposerNodeInfo

# Name of the Task
string name

# Unique ID generated for the Task by Taskflow
string uuid

# Inbound Edges
string[] inbound_edges

# Outbound Edges
string[] outbound_edges

# Input keys
string[] input_keys

# Output keys
string[] output_keys

# Value returned from the Task on completion
int32 return_value

# Debug message
string message

# Time elapsed
float64 elapsed_time


