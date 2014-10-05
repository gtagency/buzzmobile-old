#!/usr/bin/env python
import rospy
import cv
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pub_image.srv import PublishImage

image_pub = None

bridge = CvBridge()

def publish_from_file(filename):
    print "Publishing", filename, cv2.imread(filename)
    global bridge,image_pub
    cv_image = cv.fromarray(cv2.imread(filename))
    image_pub.publish(bridge.cv_to_imgmsg(cv_image, "bgr8"))
    print filename, "published."
    
def handle_publish_image(req):
    publish_from_file(req.filename)

def node():
    global image_pub
    rospy.init_node('pub_image')
    filename = rospy.get_param("~image_file", None)
    s = rospy.Service('publish_image', PublishImage, handle_publish_image)
    image_pub = rospy.Publisher('image_raw', Image, latch=True)
    print "Ready to publish images."
    rospy.Rate(1).sleep()
    if filename:
        publish_from_file(filename)
    rospy.spin()

if __name__ == '__main__':
    node()
