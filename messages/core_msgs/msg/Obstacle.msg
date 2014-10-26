#
# Obstacle messages are used to report any non-drivable object or region
# detected at a given time.
#
#

# A set of polygons to describe the obstacle.  It is assumed that
# the walls and interior of all of the polygons are non-drivable.
geometry_msgs/Polygon[] polygons
