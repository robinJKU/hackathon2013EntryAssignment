#include <ros/ros.h>

//This header allows you to publish and subscribe pcl::PointCloud<T> objects as ROS message
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

sensor_msgs::PointCloud2 cloud_transformed, cloud_income;
bool is_cloud_subscribed;
std::string target_frame_name;

void cmdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	cloud_income = *msg;
	is_cloud_subscribed = true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "point_cloud_trans");
	if (argc != 2){ROS_ERROR("need target frame name as argument"); return -1;};
	target_frame_name = argv[1];
  
	is_cloud_subscribed = false;
	ros::NodeHandle n;
	ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 100); //params: topic, size of the message queue
  
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
	ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 100, cmdCallback);
	tf::TransformListener tf_listener;
  
	ros::Rate rate(5.0);
	while(n.ok()){
		ros::spinOnce(); // for callbacks
		if(is_cloud_subscribed){
			try{
				// wait for the transform of the pointcloud (error message "Lookup would require extrapolation into the past" always comes once!)
				tf_listener.waitForTransform(target_frame_name, cloud_income.header.frame_id, cloud_income.header.stamp, ros::Duration(0.5));

				//bool 	transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
				pcl_ros::transformPointCloud (target_frame_name,cloud_income,cloud_transformed,tf_listener);
				point_cloud_pub.publish(cloud_transformed);

			}
			catch(tf::TransformException e){
				ROS_INFO("Transform_PointCloud: %s", e.what());
			}
		}
		rate.sleep();
	} 
}





