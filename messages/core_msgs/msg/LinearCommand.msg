# Bill Mania <bmania@coroware.com>
#
# Message used to send linear movement command to the linear actuator
# via the Phidgets 1065 Motor controller.  This node assumes a linear
# actuator with feedback
#
# speed is a signed integer which specifies the speed in which
# to move the linear actuator 0 indicates full stop. positive
# integers indicate a forward rotation and larger integers
# indicate a higher rotational speed.
#
int16 speed
#
# when at least one of leftSpeed or rightSpeed is non-zero,
# secondsDuration indicates for how many seconds to rotate those
# wheels at the requested speed. when secondsDuration is zero,
# the wheels will be rotated at the requested speed, in the
# requested direction until another request is received.
#
uint16 secondsDuration
#
# the number of seconds over which to effect the change in
# rotational speed from the current speed to the speed requested.
# a value of zero indicates "as fast as possible". the greater
# this value, the greater amount of time taken to change the
# speed.
uint16 acceleration
