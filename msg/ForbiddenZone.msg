# A polygonal keep-out zone in the world frame (XY footprint, all height levels).
# At least 3 vertices; the polygon may be concave but should not self-intersect.
# Traversability nodes whose XY position lies inside the polygon are marked as
# obstacles for planning.
std_msgs/Header header
geometry_msgs/Point[] vertices
