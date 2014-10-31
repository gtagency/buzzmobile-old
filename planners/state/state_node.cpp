/**
 * state_node is a simple state machine to reliably control the
 * buzzmobile's state
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// Global variables
ros::NodeHandle n;

int main(int argc, char** argv) {

  ros::init(argc, argv, "state");

  ros::Subscriber obstacles = n.subscribe("/obstacles", 1000, 0

}
