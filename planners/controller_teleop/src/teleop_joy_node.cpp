#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <core_msgs/MotorCommand.h>
#include <core_msgs/LinearCommand.h>
//#include "controller_teleop/LinearCommand.h"
//#include <corobot_msgs/takepic.h>
//#include <corobot_msgs/PanTilt.h>
//#include <corobot_msgs/velocityValue.h>
#include <sound_play/SoundRequest.h>
#include <math.h>

using namespace core_msgs;
//using namespace corobot_srvs;
/*
ros::ServiceClient moveArm_client;
ros::ServiceClient moveWrist_client; 
ros::ServiceClient moveGripper_client; 
ros::ServiceClient resetArm_client; 
*/
ros::Publisher driveControl_pub,steerControl_pub; //takepic_pub,pan_tilt_control;
int maxFwdSpeed = 100;

bool obstacleFlag = false;

ros::Publisher horn_pub;

int /*speed_left, speed_right,*/ speed_value;
bool turningLeft, turningRight;
//int pan_value,tilt_value;
//double orx;
//double ory,orz;
//int gripper_state; //0 = open, 1 = closed
//int save_image_state = 0;

/*void velocityCallback(const velocityValue::ConstPtr& msg) {
	speed_value = msg->velocity;
}
*/
void handleDrive(const sensor_msgs::Joy::ConstPtr& joy);
void handleManualToggle(const sensor_msgs::Joy::ConstPtr& joy);
void handleTurn(const sensor_msgs::Joy::ConstPtr& joy);
void handleHorn(const sensor_msgs::Joy::ConstPtr& joy);

void honkHorn(); 
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	handleHorn(joy);
	
//	if (obstacleFlag) {
//		return;
//	}
    
	handleManualToggle(joy);
    handleDrive(joy);
    handleTurn(joy);
}

int lastSpeed = -1;
void handleDrive(const sensor_msgs::Joy::ConstPtr& joy) {
    //********************************************
    //Motor control
    printf("WOAH\n"); 
    float speed = 0.0;
    int secondsDuration = 0;
    //if the stop button isnt held, we want to go
    //otherwise, just keep sending the stop command
#if 0
    float angle = atan2(joy->axes[1], -joy->axes[0]);
        ROS_INFO("angle: [%f]", angle);
    if(joy->buttons[8]) {// && (angle > (M_PI / 4)) && (angle <= (3 * M_PI / 4))) {
        speed = maxFwdSpeed * sqrt(joy->axes[0] * joy->axes[0] + joy->axes[1] * joy->axes[1]) * angle / abs(angle);
        ROS_INFO("speed: [%d]", (int)speed);

        if(speed > maxFwdSpeed) {
            speed = maxFwdSpeed;
        }

        if(speed < -maxFwdSpeed) {
            speed = -maxFwdSpeed;
        }
        secondsDuration = 0;
    }
#endif
    float correction = -1.0; //required because the range of a button axis is 0 to -1.0 (all pushed in)
    speed = correction * (joy->buttons[11] ? -1 : 1) * maxFwdSpeed * joy->axes[13];
	printf("Speed: %f\n", speed);
    if (lastSpeed != speed) {
        core_msgs::MotorCommand msg;
        msg.leftSpeed = (int)speed;
        msg.rightSpeed = (int)speed;
        msg.secondsDuration = secondsDuration;
        msg.acceleration = 100;
        driveControl_pub.publish(msg);
        lastSpeed = speed;
    }
}

void handleManualToggle(const sensor_msgs::Joy::ConstPtr& joy) {
    //NO OP for now
}

int lastSteerSpeed = -1;
void handleTurn(const sensor_msgs::Joy::ConstPtr& joy) {

    int maxSpeed = 200;
    int secondsDuration = 0;
    int speed = 0;
    //compute position as number from 0 to 1...the linear
    // pid will handle the acutal position
    float mag = sqrt(joy->axes[0] * joy->axes[0] + joy->axes[1] * joy->axes[1]);
    //joystick is around the center...send 0 speed
    if (mag > 1e-6) {
        float angle = atan2(fabs(joy->axes[1]), -joy->axes[0]);
        float position = 1.0; //1.0 w/o adjustment is neutral steering
        ROS_INFO("angle: [%f]", angle);
        //angles >= 0 are forward angles
        //if (angle >= 0) {
            position = angle * 2.0 / M_PI;
            ROS_INFO("position: [%f]", position);
        //} else { //reverse angles
            //this needs to be reflected about y axis, because we're going backwards
        //    position = (angle + M_PI) * 2.0 / M_PI;
        //    ROS_INFO("rev position: [%f]", position);
       // }
        speed = ceil((position - 1.0) * maxSpeed);
    }

    if (lastSteerSpeed != speed) {
        core_msgs::LinearCommand msg;
        msg.speed = speed;
        msg.secondsDuration = secondsDuration;
        msg.acceleration = 100;
        steerControl_pub.publish(msg);
        lastSteerSpeed = speed;
    }
}

void handleHorn(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[14]) {
        honkHorn();
    }	
}

void honkHorn() {
    sound_play::SoundRequest msg;
    msg.sound = sound_play::SoundRequest::PLAY_FILE;
    msg.command = sound_play::SoundRequest::PLAY_ONCE;
    msg.arg = "/home/agency/Downloads/vehicle042.wav";
    horn_pub.publish(msg);
}
/*
//*********************************************************
//Take picture
 if(joy->buttons[11]) // right Stick click Take Picture
  {
    if(save_image_state == 0)
    {
        takepic msg;
        msg.camera_index = 1;
        msg.take = true;
        takepic_pub.publish(msg);
	save_image_state = 1;
    }
  }
  else
	save_image_state = 0;

//********************************************************
// Pan Tilt Control	
  if(joy->axes[2]>0.5) // Pan control
  {
	if(pan_value > -70)
	{
    	pan_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[2]<-0.5) // Pan control
  {
	if(pan_value < 70)
	{
    	pan_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[3]<-0.5) // Tilt control
  {
	if(tilt_value > -30)
	{
    	tilt_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[3]>0.5) // Tilt control
  {
	if(tilt_value < 30)
	{
    	tilt_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

 if(joy->buttons[9]) // PTZ reset
  {
    tilt_value = 0;
    pan_value = 0;
    PanTilt msg;
    msg.pan = pan_value;
    msg.tilt = tilt_value;
    msg.reset = 0;
    pan_tilt_control.publish(msg);
  }

//*****************************************************

//*****************************************************
//Arm control
if(joy->axes[4]>0.5) // Shoulder control
  {
	if(ory > 0.7)
	{
    	ory -= M_PI/8;

		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[4]<-0.5) // Shoulder control
  {
    
	if( ory < 2)
	{
		ory += M_PI/8;

		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[5]<-0.5) // Elbow control
  {
    
    if( orz > 0.8)
    {
		orz -= M_PI/8;
		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[5]>0.5) // Elbow control
  {
    
    if( orz < 2.5)
    {
		orz += M_PI/8;

	   corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

 if(joy->buttons[7]) // arm reset
  {
    ory = orz = M_PI/2;

   corobot_srvs::MoveArm srv1;

    srv1.request.gripper = 0;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    resetArm_client.call(srv1);
  }
//******************************************
//wrist control
 if(joy->buttons[4]) // Wrist Left
  {
    
    if(orx < 4.5)
    {
	  orx += 0.5;

	   corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveWrist_client.call(srv1);
	}
  }

 if(joy->buttons[5]) // Wrist Right
  {
    if(orx > 0.1)
    {
	  orx -= 0.5;

   corobot_srvs::MoveArm srv1;

    srv1.request.gripper = 0;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    moveWrist_client.call(srv1);
	}
  }
//****************************************************
//gripper control
  if(joy->buttons[6])
  {
   corobot_srvs::MoveArm srv1;

    if(gripper_state == 0)
	gripper_state = 1;
    else if(gripper_state == 1)
	gripper_state = 0;
    srv1.request.gripper = gripper_state;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    moveGripper_client.call(srv1);
  }

//*****************************************************
  //if(joy->axes[1]>0)
  //{twist.linear.x = speed;twist.angular.z = 1;} // Go Foward

  //if(joy->axes[1]<0)
  //{twist.linear.x = -1*speed;twist.angular.z = -1;} // Go Backward

  //if((joy->buttons[0]==1)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0))
  //{twist.linear.x = 0;twist.angular.z = 0;}//stop

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==1)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{twist.linear.x = 0;twist.angular.z = 1;}//turn left (constant speed)

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==1)&&(joy->axes[1]==0))
  //{twist.linear.x = 0;twist.angular.z = -1;}//turn right
 
  //if((joy->buttons[0]==0)&&(joy->buttons[1]==1)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{speed = speed - 0.05;} //decrease speed

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==1)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{speed = speed + 0.05;} //decrease speed
}
*/

int obstacleCallback(const std_msgs::Bool::ConstPtr& flag) {
	if (!obstacleFlag && flag->data) {
		honkHorn();
	} //TODO: for some reason this will honk repeatedly (repeated obstacles)...may need to debug this
	
	obstacleFlag = flag->data;
}

int main(int argc, char** argv) {
 // pan_value = 0;
  //tilt_value = 0;
  //orx = 2.3;//initial value for wrist
  //ory = M_PI/2;
  //orz = M_PI/2;
  //gripper_state = 0;
  //speed_value = 75;
  //turningLeft = false;
  //turningRight = false;

  ros::init(argc, argv, "teleop_joy");
  
  ros::NodeHandle n;
 
  /*ros::Subscriber sub = n.subscribe<joy::Joy>("joy", 1000, joyCallback);*/

/*  moveArm_client = n.serviceClient<MoveArm>("move_arm");
  moveWrist_client = n.serviceClient<MoveArm>("move_wrist");
  moveGripper_client = n.serviceClient<MoveArm>("move_gripper");
  resetArm_client = n.serviceClient<MoveArm>("reset_arm"); 
*/

  driveControl_pub = n.advertise<core_msgs::MotorCommand>("PhidgetMotor", 100);
  steerControl_pub = n.advertise<core_msgs::LinearCommand>("PhidgetLinear", 100);
  horn_pub = n.advertise<sound_play::SoundRequest>("robotsound", 100);

  //steerControl_pub = n.advertise<core_msgs::LinearCommand>("LinearPID", 100);
/*  takepic_pub = n.advertise<takepic>("takepicture",100);
  pan_tilt_control = n.advertise<PanTilt>("pantilt",10);
*/
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
  ros::Subscriber sub2 = n.subscribe<std_msgs::Bool>("obstacle_flag", 1000, obstacleCallback);
 // ros::Subscriber velocity = n.subscribe<velocityValue>("velocityValue", 1000, velocityCallback);

  ros::spin();
}
