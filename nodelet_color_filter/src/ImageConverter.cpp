#include <ros/ros.h>
#include<nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

namespace nodelet_color_filter {
  class ImageConverter : public nodelet::Nodelet
{
  public:
  ImageConverter() {};
  
  ~ImageConverter() {}
  
  virtual void onInit();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  int color_;
  
};

}; //color_filter_nodelet<

#include<pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(nodelet_color_filter,ImageConverter,nodelet_color_filter::ImageConverter,nodelet::Nodelet);

namespace nodelet_color_filter 
{
  void ImageConverter::onInit() 
  {
   ros::NodeHandle& private_nh = getPrivateNodeHandle();
   private_nh.getParam("color",color_);
   image_sub_ = private_nh.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
   image_pub_ = private_nh.advertise<sensor_msgs::Image>("/camera/red/image_raw", 1);
  
  }
  
  void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /* Publishes red channel
     * Mat image: saves image to be converted
     * Mat channel[3]: Matrices for RGB
     * CvImage out_msg: contains output data
    */
    
    cv::Mat image = (*cv_ptr).image;     
    cv::Mat channel[3];
    
    // Splits image into single channels
    split(image,channel);
    
    // Build output messsage
    cv_bridge::CvImage out_msg;
    out_msg.header = cv_ptr->header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_msg.image = channel[2];
        
    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nodelet_color_filter");
  nodelet_color_filter::ImageConverter ic;
  ros::spin();
  return 0;
}