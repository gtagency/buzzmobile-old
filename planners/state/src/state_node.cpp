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

ros::NodeHandle n;
ros::Publisher state_publisher;

StateEnum state = MANUAL;
bool braked = false;

void brakeCallback(const std_msgs::Bool::ConstPtr& brake) {
  if (brake->data) {
    state = BRAKE;
  }
  core_msgs::State msg;
  msg.state = state;
  state_publisher.publish(msg);
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

    core_msgs::State msg;
    msg.state = state;
    state_publisher.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state");

  ros::Subscriber joy = n.subscribe<sensor_msgs::Joy>("joy", 1000, joystickCallback);
  ros::Subscriber brake = n.subscribe<std_msgs::Bool>("brake", 1000, brakeCallback);

  state_publisher = n.advertise<core_msgs::State>("state", 1000);

}
