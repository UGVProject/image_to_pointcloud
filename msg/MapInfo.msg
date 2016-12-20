# This hold basic information about the characterists of the ni-slam map

Header header
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
int32 width
# Map height [cells]
int32 height
# transmitter status
int8 status
# The origin of the map.
geometry_msgs/Pose origin