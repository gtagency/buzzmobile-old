#include <ros/ros.h>
#include <std_msgs/Bool.h>


std_msgs::Bool on;
std_msgs::Bool off;
ros::Publisher car_horn_pub;

void honkLongHandler(const std_msgs::Bool::ConstPtr&) {
//  std::cout << "In honkLong" << std::endl;
  car_horn_pub.publish(on);
  ros::Duration(0.5).sleep(); // sleep for half a second
  car_horn_pub.publish(off);
//  std::cout << "Exiting honkLong" << std::endl;
}

void honkShortHandler(const std_msgs::Bool::ConstPtr&) {
//  std::cout << "In honkShort" << std::endl;
  car_horn_pub.publish(on);
  ros::Duration(0.125).sleep(); // sleep for half a second
  car_horn_pub.publish(off);
//  std::cout << "Exiting honkShort" << std::endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "horn");
  ros::NodeHandle n;

  //because they don't accept constructor ARGHHHs!
  on.data = true;
  off.data = false;

  ros::Subscriber s = n.subscribe<std_msgs::Bool>("honk_long", 1, honkLongHandler);
  ros::Subscriber s2 = n.subscribe<std_msgs::Bool>("honk_short", 1, honkShortHandler);

  car_horn_pub = n.advertise<std_msgs::Bool>("car_horn", 1);
  ros::spin();
  return 0;
}
