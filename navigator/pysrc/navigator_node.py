#!/usr/bin/env python
#

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from road_network import *
from navigator.srv import *

inf = float("inf")

class Navigator(object):

    def __init__(self, networkFileName):

        rospy.init_node('navigator_node')

        self.roadNetwork = RoadNetwork(networkFileName)
        self.curWaypointPub = rospy.Publisher("current_waypoint", Point)
        rospy.Subscriber("fix", NavSatFix, self.fixUpdated)
        rospy.Service("setDestination", Destination, self.setDestination)

        self.latitude = inf
        self.longitude = inf
        self.route = []

    def nearCurrentWaypoint(self):
        # test if we're within the error bounds of the waypoint..if so, we're 'near' it
        # SUPER CHEAT because I know the GPS data is coming from a phone, and therefore
        # I don't really have a full covariance matrix.  This pulls out the 68% accurac
        # computed by the phone, which is 1 std dev from the estimated position
        # TODO Need to figure out how to compute error better, for nearCurrentWaypoint
        errorInMeters = sqrt(self.covariance[0])
        return euclidean(self.curWaypoint.latitude, self.curWaypoint.longitude, self.latitude, self.longitude) < errorInMeters

    def reroute(self, destLat, destLon):
        if self.latitude != inf and self.longitude != inf:
            self.route = self.roadNetwork.findPathFromPosition(self.latitude, self.longitude, destLat, destLon)
            self.updateAlongRoute(True)
        else: 
            rospy.loginfo("No Nav fix received, cannot route to destination")
            
    def setDestination(self, req):
        self.reroute(req.destination.x, req.destination.y)

    def fixUpdated(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.covariance = data.covariance #3x3 matrix in a 9 element array
        rospy.loginfo("Updated position: %f, %f" % (self.latitude, self.longitude))
        self.updateAlongRoute(self.nearCurrentWaypoint())

    def updateAlongRoute(self, pushNext):
        if pushNext:
            print "About to publish update message"
            self.curWaypoint = self.route.pop(0)
            msg = Point()
            msg.x = self.curWaypoint.latitude
            msg.y = self.curWaypoint.longitude
            msg.z = 0
            print "Publishing"
            self.curWaypointPub.publish(msg)


def main():
    node = Navigator('/home/viki/catkin_ws/src/buzzmobile/navigator/data/parade_route_graph.json')
    
    rospy.spin()

if __name__ == "__main__":
    main()
