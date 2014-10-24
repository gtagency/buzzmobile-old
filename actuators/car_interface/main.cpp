#include <ros/ros.h>
#include "arduino.h"
#include <core_msgs/Odom.h>
#include <core_msgs/MotionCommand.h>

using namespace std;

const double ticksPerRev = 3600;
const double wheelCirc = 2.198;

ros::Publisher encoder_pub;
ros::Subscriber command_sub;

Arduino arduino;

int odom_sequence = 0;

ros::Time last_command_time;

void encoder_callback(int);

void command_callback(car::MotionCommand);

int main(int argc, char **argv) {

  ros::init(argc, argv, "car_interface");

  arduino.open("/dev/arduino_motor_controller", 9600);
  arduino.setEncoderCallback(encoder_callback);

  ros::NodeHandle node_handle;

  last_command_time = ros::Time::now();

  encoder_pub = node_handle.advertise<car::Odom>("encoder_odom", 1000);

  command_sub = node_handle.subscribe("motor_commands", 1000, command_callback);

  while(ros::ok()) {
    ros::spinOnce();
    // 1 sec command frequency required to maintain velocity
    if((ros::Time::now() - last_command_time).toSec() > 1.0) {
      arduino.setSpeed(0);
    }
  }
  ros::spin();

  return 0;
}

void encoder_callback(int tickCount) {
  car::Odom msg;
  msg.distance_travelled = (tickCount * wheelCirc) / ticksPerRev;
  msg.header.seq = odom_sequence++;
  msg.header.stamp = ros::Time::now();
  encoder_pub.publish(msg);
}

void command_callback(car::MotionCommand cmd) {
  arduino.setSpeed(cmd.speed);
  arduino.setSteering(cmd.angle);
  last_command_time = cmd.header.stamp;
}
