# This message presents the current state of a CiA402 drive on multiple DoFs.

# Drive States
uint8 STATE_UNDEFINED=0
uint8 STATE_START=1
uint8 STATE_NOT_READY_TO_SWITCH_ON=2
uint8 STATE_SWITCH_ON_DISABLED=3
uint8 STATE_READY_TO_SWITCH_ON=4
uint8 STATE_SWITCH_ON=5
uint8 STATE_OPERATION_ENABLED=6
uint8 STATE_QUICK_STOP_ACTIVE=7
uint8 STATE_FAULT_REACTION_ACTIVE=8
uint8 STATE_FAULT=9

# Modes of Operation
uint8 MODE_NO_MODE=0
uint8 MODE_PROFILED_POSITION=1
uint8 MODE_PROFILED_VELOCITY=3
uint8 MODE_PROFILED_TORQUE=4
uint8 MODE_HOMING=6
uint8 MODE_INTERPOLATED_POSITION=7
uint8 MODE_CYCLIC_SYNC_POSITION=8
uint8 MODE_CYCLIC_SYNC_VELOCITY=9
uint8 MODE_CYCLIC_SYNC_TORQUE=10

std_msgs/Header header

# DoF name
string[] dof_names

# Current state of the drive
string[] drive_states

# Current mode of operation
string[] modes_of_operation

# Current Status Word
int16[] status_words
