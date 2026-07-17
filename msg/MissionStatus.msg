std_msgs/Header header

uint8 IDLE=0
uint8 READY=1
uint8 PLANNING=2
uint8 AWAITING_APPROVAL=3
uint8 EXECUTING=4
uint8 PAUSED=5
uint8 COMPLETED=6
uint8 FAILED=7
uint8 ABORTED=8

uint8 state
string state_name
string summary
string failure_reason
uint32 current_segment
uint32 total_segments
float32 progress
float32 distance_remaining
float32 estimated_seconds_remaining
float32 current_speed
string motion_mode
bool route_valid
bool can_resume
