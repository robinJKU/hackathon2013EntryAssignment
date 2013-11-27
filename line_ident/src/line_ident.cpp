#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class LineIdentifier {
	
	ros::NodeHandle n;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	image_transport::Publisher image_pub;
	int th_canny;
	int th_hough;
	bool debug_msg;
	bool lines_only;
	
	public:
	LineIdentifier() : it(n) {
		
		n.param("/line_ident/th_canny", th_canny, 100);
		n.param("/line_ident/th_hough", th_hough, 150);
		n.param("/line_ident/debug_msg", debug_msg, false);
		n.param("/line_ident/lines_only", lines_only, false);
		
		if(debug_msg) ROS_INFO("Params are: %d, %d", th_canny, th_hough);
		
		image_pub = it.advertise("camera/lines/image_raw", 1);
		image_sub = it.subscribe("camera/rgb/image_raw", 1, &LineIdentifier::convert, this);
		
		//cv::namedWindow("window");
		
	}
	
	void convert(const sensor_msgs::ImageConstPtr& msg) {
		
		cv_bridge::CvImagePtr cv_ptr;
		
		// image conversion
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			//cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
			if(debug_msg) ROS_INFO("Image converted...");
		} catch(cv_bridge::Exception& e) {
			ROS_ERROR("Image conversion exception: %s",e.what());
			return;
		}
		
		cv::Mat gray, edges, lines;		
		cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
		
		// Canny algorithm
		try {		
			cv::Canny(gray, edges, th_canny, 3*(th_canny), 3);
			if(debug_msg) ROS_INFO("Edges detected...");
		} catch(...) {
			ROS_ERROR("Canny algorithm exception...");
			return;
		}
		
		// Hough transformation
		std::vector<cv::Vec2f> vec_lines;
		cv::HoughLines(edges, vec_lines, 1, CV_PI/180, th_hough, 0, 0);
		
		if(lines_only) {
			cv_ptr->image = cv::Scalar(255,255,255);
		}
		
		// print lines
		for(size_t i = 0; i < vec_lines.size(); i++) {
			float rho = vec_lines[i][0], theta = vec_lines[i][1];
			cv::Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line(cv_ptr->image, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
		}
		
		//cv::imshow("window", cv_ptr->image);
		//cv::waitKey(500);
		
		image_pub.publish(cv_ptr->toImageMsg());
		
	}
};

int main(int argc, char **argv) {
	
  ros::init(argc, argv, "line_ident");
	
	LineIdentifier li;
	
  ros::spin();
	
  return 0;
	
}
