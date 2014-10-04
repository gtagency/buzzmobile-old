#
# Obstacle messages are used to report any non-drivable object or region
# detected at a given time.
#
#

# Generic obstacle to be avoided
uint8 TYPE_OBSTACLE = 1
# A lane, which defines a hard boundary for the road
uint8 TYPE_LANE = 2
# A soft lane, which is a lane boundary that the car may cross
# NOTE: NOT YET USED
uint8 TYPE_SOFT_LANE = 3

# Type of obstacle being reported
uint8 type

# A set of polygons to describe the obstacle.  It is assumed that
# the walls and interior of all of the polygons are non-drivable.
geometry_msgs/Polygon[] polygons
