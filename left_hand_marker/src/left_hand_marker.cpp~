#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "my_hand_marker");
  // Set Markernode and publish rate 100Hz
  ros::NodeHandle node;
  ros::Rate r(100);
  ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //  Set our initial shape type to be a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  
  tf::TransformListener listener;
  
  while (ros::ok())
  {
    // Initialize Transformation
    tf::StampedTransform transform;
    // Get Transformation
    // Alternative code to prevent an error "Lookup woult require extrapolation into the past"
    try 
    {
      listener.waitForTransform("/openni_depth_frame" ,"/left_hand_1", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("/openni_depth_frame" ,"/left_hand_1" , ros::Time(0), transform);
    
    } 
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
    
    // Initialize Marker
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
   // marker.header.frame_id = "/openni_depth_frame";
    marker.header.frame_id = "left_hand_1";
    marker.header.stamp = ros::Time::now();
   
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "my_hand_marker";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    
    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0; 
    marker.pose.orientation.y = 0; 
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    
    // Set the scale of the marker
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    // Publish the marker
    marker_pub.publish(marker);
    // Call Callbacks for good measure 
    ros::spinOnce();
    
    r.sleep();
  }
}