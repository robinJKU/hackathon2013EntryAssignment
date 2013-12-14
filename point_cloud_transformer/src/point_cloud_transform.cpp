#include <ros/ros.h>

//This header allows you to publish and subscribe pcl::PointCloud<T> objects as ROS message
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>

namespace point_cloud_transformer{

class PC2TransformerNodelet : public nodelet::Nodelet{
	public:
		sensor_msgs::PointCloud2 cloud_transformed, cloud_income;
		bool is_cloud_subscribed;
		std::string target_frame_name;


		PC2TransformerNodelet() {}; //:is_cloud_subscribed(false)
		~PC2TransformerNodelet() {}

		void cmdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
			cloud_income = *msg;
			is_cloud_subscribed = true;
		}
  private:
	
		virtual void onInit(){

		ros::NodeHandle n = this->getPrivateNodeHandle();
		is_cloud_subscribed = false;
		ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 100); //params: topic, size of the message queue
		
		// Create a subscriber.
		// Name the topic, message queue, callback function with class name, and object containing callback function.
		ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 100, &PC2TransformerNodelet::cmdCallback, this);
		tf::TransformListener listener; ///cloud_throttled
		
		ros::Rate rate(5.0);
		while(n.ok()){
		    ros::spinOnce(); // for callbacks
		    if(is_cloud_subscribed){
		    	try{
		       pcl_ros::transformPointCloud ("target_frame",cloud_income,cloud_transformed,listener); 
		       //target_frame_name
		       point_cloud_pub.publish(cloud_transformed);
		    	 //   cloud_subscribed = false;
		    	}
		    	catch(tf::TransformException e){
		    		ROS_INFO("Transform_PointCloud: %s", e.what());
		    	}
		    }

		  rate.sleep();
		}
	}
};

#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(point_cloud_transformer, PC2TransformerNodelet, point_cloud_transformer::PC2TransformerNodelet, nodelet::Nodelet) 

PLUGINLIB_EXPORT_CLASS(point_cloud_transformer::PC2TransformerNodelet,
                       nodelet::Nodelet);
}


