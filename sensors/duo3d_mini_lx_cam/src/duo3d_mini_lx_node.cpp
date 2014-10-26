
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

ros::Publisher pc_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
//ros::Publisher imu_pub;
ros::Publisher angles_pub;

//TODO: in callbacks, check to see if node has subscribers and publish
//e.g.
//if (pc_pub.getNumSubscribers() > 0) {
//  GENERATE POINTCLOUD
//  pc_pub.publish(pointCloud);
//}

int main(int argc, char **argv) {

  ros::NodeHandle n;

  pc_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 100);
  left_image_pub = n.advertise<sensor_msgs::Image>("left/image_raw", 100);
  right_image_pub = n.advertise<sensor_msgs::Image>("right/image_raw", 100);
 // imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  // Euler angles, x = roll, y = pitch, z = yaw
  angles_pub = n.advertise<geometry_msgs::Vector3>("angles", 100);
  
  ros::spin();
  return 0;
}
