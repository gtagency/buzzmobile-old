/**
 * The state node acts as a global state machine to control the state of the
 * entire system
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <core_msgs/State.h>
#include "ps3buttons.h"

enum StateEnum { AUTO = 0, MANUAL = 1, START = 2, BRAKE = 3 };

ros::Publisher state_pub;
ros::Publisher honk_long_pub;
ros::Publisher honk_short_pub;

StateEnum state = MANUAL;

void publishState() {
  core_msgs::State msg;
  msg.state = state;
  state_pub.publish(msg);
}

void publishHornLong() {
  std_msgs::Bool msg;
  msg.data = true;
  honk_long_pub.publish(msg);
}

void publishHornShort() {
  std_msgs::Bool msg;
  msg.data = true;
  honk_short_pub.publish(msg);
}

void brakeCallback(const std_msgs::Bool::ConstPtr& brake) {
  if (brake->data && state == AUTO) {
    state = BRAKE;
    publishState();
    publishHornLong();
  }
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick) {
  switch(state) {
  case MANUAL:
    if (joystick->buttons[PS3_BUTTON_ACTION_TRIANGLE]) {
      state = START;
      break;
    }
    break;
  case START:
    if (joystick->buttons[PS3_BUTTON_ACTION_CIRCLE]) {
      state = MANUAL;
      break;
    } else if (joystick->buttons[PS3_BUTTON_START]) {
      publishHornShort();
      state = AUTO;
      break;
    }
    break;
  case AUTO:
    if (joystick->buttons[PS3_BUTTON_ACTION_CIRCLE]) {
      state = MANUAL;
      break;
    }
    break;
  case BRAKE:
    if (joystick->buttons[PS3_BUTTON_ACTION_CIRCLE]) {
      state = MANUAL;
    }
  }

  publishState();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state");
  ros::NodeHandle n;

  ros::Subscriber joy = n.subscribe<sensor_msgs::Joy>("joy", 1000, joystickCallback);
  ros::Subscriber brake = n.subscribe<std_msgs::Bool>("brake", 1000, brakeCallback);

  honk_long_pub = n.advertise<std_msgs::Bool>("honk_long", 1);
  honk_short_pub = n.advertise<std_msgs::Bool>("honk_short", 1);
  state_pub = n.advertise<core_msgs::State>("state", 1000, true);

  publishState();
  ros::spin();
  return 0;
}
