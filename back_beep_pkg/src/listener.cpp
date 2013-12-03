#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sound_play/SoundRequest.h"

ros::Publisher rs_pub;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	sound_play::SoundRequest msgSound;
	msgSound.sound = -2;
	msgSound.command = 2;
	msgSound.arg = "/home/rupertschlager/back_beep/src/back_beep_pkg/sounds/say-beep.wav";
	if(msg->linear.x<0){
		ROS_INFO("Negative Velocity: [%f]", msg->linear.x);
		rs_pub.publish(msgSound);
	}
	if(msg->linear.x>0){
		ROS_INFO("Positive Velocity: [%f]", msg->linear.x);
	}
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::NodeHandle rs;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


 

  rs_pub = rs.advertise<sound_play::SoundRequest>("/robotsound", 1000);
  
  ros::spin();

  return 0;
}
