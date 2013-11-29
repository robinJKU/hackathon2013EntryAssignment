/* 
 * Author: Lukas Schwarz
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nodelet/nodelet.h>

class ImageConverter{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
  
	public:
		ImageConverter(): it_(nh_){
			// Subscrive to input video feed and publish output video feed
			image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
			image_pub_ = it_.advertise("/camera/depth_narrow/image_raw", 1);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			
			// convert ROS => openCV
			try{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e){
			  ROS_ERROR("cv_bridge exception (ROS => openCV): %s", e.what());
			  return;
			}			
			
			// grayscale
			int helpVal;
			for(int i=0; i<cv_ptr->image.rows; i++) {
				for(int j=0; j<cv_ptr->image.cols; j++) {
					helpVal = (cv_ptr->image.at<cv::Vec3b>(i,j)[0] + cv_ptr->image.at<cv::Vec3b>(i,j)[1] + cv_ptr->image.at<cv::Vec3b>(i,j)[2])/3;
					cv_ptr->image.at<cv::Vec3b>(i,j)[0] = helpVal; 
					cv_ptr->image.at<cv::Vec3b>(i,j)[1] = helpVal; 
					cv_ptr->image.at<cv::Vec3b>(i,j)[2] = helpVal;  
			   }
			}
			
			
			// Output modified video stream openCV => ROS
			image_pub_.publish(cv_ptr->toImageMsg());
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
