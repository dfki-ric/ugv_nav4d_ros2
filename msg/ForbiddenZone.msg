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
# TRAVERSABLE = manual MLS edit: patches inside the polygon (level band) are
# deleted from the planner's MLS before traversability expansion; if
# fill_enabled, the area is refilled with synthetic ground patches so the
# planner can drive through (tall grass, sensor artifacts).
uint8 TRAVERSABLE=6
uint8 zone_type
string label
float32 cost_multiplier
float32 speed_limit
float32 preferred_heading
builtin_interfaces/Time expires_at

# TRAVERSABLE zones only: synthetic fill plane through the polygon centroid
# at (mean vertex z + fill_z_offset), tilted by fill_roll/fill_pitch
# (radians, about the world X/Y axes at the centroid).
bool fill_enabled
float32 fill_z_offset
float32 fill_roll
float32 fill_pitch
# Deletion ceiling: patches whose top lies more than delete_top above the
# highest clicked vertex survive (protects overhead structures beyond robot
# height). 0 means the default of 2.0 m. The floor is fixed 2.0 m below the
# lowest vertex.
float32 delete_top
