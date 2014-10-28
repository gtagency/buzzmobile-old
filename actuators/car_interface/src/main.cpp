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

ros::Duration keep_alive_frequency(1.0);

void odometry_callback(int, float);

void command_callback(core_msgs::MotionCommand::ConstPtr);

void keep_alive_callback(const ros::TimerEvent&);

int main(int argc, char **argv) {

  ros::init(argc, argv, "car_interface");

  arduino.open("/dev/arduino_motor_controller", 9600);
  arduino.setOdometryCallback(odometry_callback);

  ros::NodeHandle node_handle;

  last_command_time = ros::Time::now();

  encoder_pub = node_handle.advertise<core_msgs::Odom>("encoder_odom", 1000);

  command_sub = node_handle.subscribe("motion_command", 1000, command_callback);

  ros::Timer keepAliveTimer = node_handle.createTimer(keep_alive_frequency, keep_alive_callback);
  ros::spin();

  return 0;
}

void keep_alive_callback(const ros::TimerEvent&) {
  ROS_INFO("Keep alive callback called");
  // 1 sec command frequency required to maintain velocity
  if((ros::Time::now() - last_command_time) > keep_alive_frequency) {
    arduino.setSpeed(0);
  }
}
void odometry_callback(int tickCount, float steeringAngle) {
  core_msgs::Odom msg;
  msg.distance_travelled = (tickCount * wheelCirc) / ticksPerRev;
  msg.steering_angle = steeringAngle;
  msg.header.seq = odom_sequence++;
  msg.header.stamp = ros::Time::now();
  encoder_pub.publish(msg);
}

void command_callback(core_msgs::MotionCommand::ConstPtr cmd) {
  ROS_INFO("Command received, speed: %f, angle: %f", cmd->speed, cmd->angle);
  arduino.setSpeed(cmd->speed);
  arduino.setSteering(cmd->angle);
  last_command_time = cmd->header.stamp;
}
