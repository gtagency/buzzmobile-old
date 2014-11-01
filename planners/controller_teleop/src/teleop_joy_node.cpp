#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <core_msgs/MotionCommand.h>
#include <core_msgs/State.h>
//#include "controller_teleop/LinearCommand.h"
//#include <corobot_msgs/takepic.h>
//#include <corobot_msgs/PanTilt.h>
//#include <corobot_msgs/velocityValue.h>
#include <sound_play/SoundRequest.h>
#include <math.h>

using namespace core_msgs;

#define reverse_button buttons[11]
#define brake_button buttons[14]
#define horn_button buttons[15]

ros::Publisher motion_pub;
ros::Publisher horn_pub;
//NOTE: this is also published by the autonomous brake...this is ok.
ros::Publisher brake_pub;

int maxFwdSpeed = 1; //m/s
int pubFreq     = 10; //hz

bool obstacleFlag = false;

float lastSpeed = 0;
float lastAngle = 0;

unsigned int state;
bool manualToggle = true; //start up with manual toggle = true
bool brakePushed  = false;
//int pan_value,tilt_value;
//double orx;
//double ory,orz;
//int gripper_state; //0 = open, 1 = closed
//int save_image_state = 0;

/*void velocityCallback(const velocityValue::ConstPtr& msg) {
	speed_value = msg->velocity;
}
*/
void handleBrake(const sensor_msgs::Joy::ConstPtr& joy);
void handleDrive(const sensor_msgs::Joy::ConstPtr& joy);
void handleTurn(const sensor_msgs::Joy::ConstPtr& joy);
void handleHorn(const sensor_msgs::Joy::ConstPtr& joy);

void honkHorn(); 
void sendMotionCommand();
void sendBrakeCommand();

void keepAliveCallback(const ros::TimerEvent&) {
  // To keep alive, just resend the motion command
  if (state == core_msgs::State::MANUAL) { //manual 
    sendMotionCommand();
  }
}

void stateCallback(const core_msgs::State::ConstPtr& stateMsg) {
  state = stateMsg->state;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

  if (state == core_msgs::State::MANUAL) { //manual 
    handleHorn(joy);
    handleDrive(joy);
    handleTurn(joy);
  }
}

void sendMotionCommand() {
  core_msgs::MotionCommand msg;
  msg.speed = lastSpeed;
  msg.angle = lastAngle;
  motion_pub.publish(msg);
}

void sendBrakeCommand() {
  std_msgs::Bool msg;
  msg.data = manualToggle;
  brake_pub.publish(msg);
}

void handleDrive(const sensor_msgs::Joy::ConstPtr& joy) {
  //********************************************
  //Motor control
  float speed = 0.0;
  //if the stop button isnt held, we want to go
  //otherwise, just keep sending the stop command
  float correction = -1.0; //required because the range of a button axis is 0 to -1.0 (all pushed in)
  speed = correction * (joy->reverse_button ? -1 : 1) * maxFwdSpeed * joy->axes[13];
	ROS_INFO("Speed: %f", speed);
  if (lastSpeed != speed) {
    lastSpeed = speed;
    sendMotionCommand();
  }
}

void handleBrake(const sensor_msgs::Joy::ConstPtr& joy) {
  //bool pub = false;
  if (joy->brake_button) {
    // When we first press the button, toggle the flag and publish
    if (!brakePushed) {
      brakePushed = true;
      manualToggle = true;
      sendBrakeCommand();
    }
  } else {
    // When we first release the button, toggle the flag and publish
    if (brakePushed) {
      brakePushed = false;
      manualToggle = false;
      sendBrakeCommand();
    }
  }
}

void handleTurn(const sensor_msgs::Joy::ConstPtr& joy) {
  float angle = 0;
  // first test is to see if we're close to 0
  float mag = sqrt(joy->axes[0] * joy->axes[0] + joy->axes[1] * joy->axes[1]);
  //joystick is around the center...send 0 speed
  if (mag > 1e-6) {
    angle = -(atan2(fabs(joy->axes[1]), -joy->axes[0]) - M_PI_2);
    ROS_INFO("angle: [%f]", angle);
  }
  if (lastAngle != angle) {
    lastAngle = angle;
    sendMotionCommand();
  }
}

void handleHorn(const sensor_msgs::Joy::ConstPtr& joy) {
  std_msgs::Bool msg;
  msg.data = joy->horn_button;
  horn_pub.publish(msg);
}

void honkHorn() {
    sound_play::SoundRequest msg;
    msg.sound = sound_play::SoundRequest::PLAY_FILE;
    msg.command = sound_play::SoundRequest::PLAY_ONCE;
    msg.arg = "/home/agency/Downloads/vehicle042.wav";
    horn_pub.publish(msg);
}

void obstacleCallback(const std_msgs::Bool::ConstPtr& flag) {
	if (!obstacleFlag && flag->data) {
		honkHorn();
	} //TODO: for some reason this will honk repeatedly (repeated obstacles)...may need to debug this
	
	obstacleFlag = flag->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joy");
  
  ros::NodeHandle n;
 
  motion_pub = n.advertise<core_msgs::MotionCommand>("motion_command", 100);
  horn_pub   = n.advertise<std_msgs::Bool>("car_horn", 1);
//  brake_pub  = n.advertise<std_msgs::Bool>("brake", 100, true);


  // Initialize the latched topic
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
  ros::Subscriber sub2 = n.subscribe<core_msgs::State>("state", 1000, stateCallback);
  //ros::Subscriber sub2 = n.subscribe<std_msgs::Bool>("obstacle_flag", 1000, obstacleCallback);
//  ros::Rate r(pubFreq);

  ros::Timer keepAliveTimer = n.createTimer(ros::Duration(1.0/pubFreq), keepAliveCallback);
  ros::spin();

  return 0;
}
