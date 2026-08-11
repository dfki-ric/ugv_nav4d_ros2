# A polygonal operational zone in the world frame. At least 3 vertices; the
# polygon may be concave but should not self-intersect.
# LEVEL SEMANTICS: the vertex z values carry the surface the zone was drawn
# on; the zone applies within [min vz, max vz] +/- 2 m, so it does not leak
# onto other storeys (bridge above a ground zone, floor below). Zones whose
# vertices all have z == 0 apply to the whole column (legacy behavior).
std_msgs/Header header
geometry_msgs/Point[] vertices

# Operational zone semantics. KEEP_OUT preserves the original behaviour.
uint8 KEEP_OUT=0
uint8 CAUTION=1
uint8 SPEED_LIMIT=2
uint8 PREFERRED=3
uint8 DIRECTION_RESTRICTED=4
uint8 ANNOTATION=5
uint8 zone_type
string label
float32 cost_multiplier
float32 speed_limit
float32 preferred_heading
builtin_interfaces/Time expires_at
