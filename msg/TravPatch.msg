std_msgs/Header header
string frame_id
float32 a
float32 b
float32 c
float32 d
geometry_msgs/Point position
std_msgs/ColorRGBA color
# Machine-readable terrain information for operator inspection and route-risk
# calculation. `type` uses the TravGenNode NodeType numeric values.
uint8 type
string type_name
float32 slope
float32 slope_direction
int32 cost
float32[] allowed_orientation_starts
float32[] allowed_orientation_widths
# Why an OBSTACLE patch was classified as one (traversability_generator3d
# ObstacleCause numeric values). 0 = NONE (not an obstacle).
uint8 obstacle_cause
string obstacle_cause_name
