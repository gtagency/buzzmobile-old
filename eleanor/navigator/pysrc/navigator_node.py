#!/usr/bin/env python
#

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from road_network import *
from navigator.srv import *

import math

inf = float("inf")

class Navigator(object):

    def __init__(self, networkFileName):

        rospy.init_node('navigator_node')

        self.roadNetwork = RoadNetwork(networkFileName)
        self.curWaypointPub = rospy.Publisher("current_waypoint", Point, latch=True)
        rospy.Subscriber("fix", NavSatFix, self.fixUpdated)
        rospy.Service("setDestination", Destination, self.setDestination)

        self.latitude = inf
        self.longitude = inf
        
        self.destLat = rospy.get_param("~destination_lat", None)
        self.destLon = rospy.get_param("~destination_lon", None)
        rospy.loginfo("Initial target: %f, %f" % (self.destLat, self.destLon))
        # NOTE: don't route right away, because we don't have a lat/long yet.  we'll
        # route once we get an update from the gps
        self.route = []
        self.curWaypoint = None

    def nearCurrentWaypoint(self):
        # test if we're within the error bounds of the waypoint..if so, we're 'near' it
        # SUPER CHEAT because I know the GPS data is coming from a phone, and therefore
        # I don't really have a full covariance matrix.  This pulls out the 68% accurac
        # computed by the phone, which is 1 std dev from the estimated position
        # TODO Need to figure out how to compute error better, for nearCurrentWaypoint
        errorInMeters = math.sqrt(self.covariance[0])
        dist = euclidean(self.curWaypoint.latitude, self.curWaypoint.longitude, self.latitude, self.longitude) * 69 * 1.6 * 1000
        rospy.loginfo("Error: %f, Dist: %f" % (errorInMeters, dist))
        return dist < errorInMeters

    def reroute(self):
        if self.latitude != inf and self.longitude != inf:
            self.route = self.roadNetwork.findPathFromPosition(self.latitude, self.longitude, self.destLat, self.destLon)
            self.updateAlongRoute(True)
        else: 
            rospy.loginfo("No Nav fix received, cannot route to destination")
            
    def setDestination(self, req):
        self.destLat = req.destination.x
        self.destLon = req.destination.y
        self.reroute()

    def fixUpdated(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.covariance = list(data.position_covariance) #3x3 matrix in a 9 element array
        rospy.loginfo("Updated position: %f, %f" % (self.latitude, self.longitude))
        if self.curWaypoint:
            self.updateAlongRoute(self.nearCurrentWaypoint())
        elif self.destLat and self.destLon:
            self.reroute()

    def updateAlongRoute(self, pushNext):
        if self.route and pushNext:
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
