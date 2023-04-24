# This message presents the current state of a CiA402 drive on multiple DoFs.

std_msgs/Header header

# DoF name
string[] dof_names

# Current state of the drive
string[] drive_states

# Current mode of operation
string[] modes_of_operation

# Current Status Word
int16[] status_words
