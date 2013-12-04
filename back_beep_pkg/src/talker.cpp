#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);
  ros::Rate loop_rate(10);

  geometry_msgs::Twist com_vel;
  com_vel.linear.x=-4.0;
  com_vel.linear.y=0.0;
  com_vel.linear.z=0.0;
  com_vel.angular.x=0.0;
  com_vel.angular.y=0.0;
  com_vel.angular.z=2.0;

  int count = 0;
  while (ros::ok())
  {
    com_vel.linear.x++;
    if(com_vel.linear.x>10){
	com_vel.linear.x=-10.0;
    }
    ROS_INFO("velocity_x=%f",com_vel.linear.x);
    chatter_pub.publish(com_vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
