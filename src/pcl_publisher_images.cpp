#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>


using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  string rgb_topic = "/pcl_publisher/rgb";
  string depth_topic = "/pcl_publisher/depth";
  string camera_info_topic = "/pcl_publisher/camera_info";
  
  nh.getParam("rgb_topic", rgb_topic);
  nh.getParam("depth_topic", depth_topic);
  nh.getParam("camera_info_topic", camera_info_topic);

  image_transport::Publisher rgb_pub = it.advertise(rgb_topic, 1);
  image_transport::Publisher depth_pub = it.advertise(depth_topic, 1);

  ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);
  
  cv::Mat rgb_image = cv::imread(
    "/home/rakesh/rakesh/cmpt726/BerkeleyMHAD/Kinect/Kin01/S01/A01/R01/kin_k01_s01_a01_r01_color_00000.ppm", 
    CV_LOAD_IMAGE_COLOR
  );

  cv::Mat depth_image = cv::imread(
      "/home/rakesh/rakesh/cmpt726/BerkeleyMHAD/Kinect/Kin01/S01/A01/R01/kin_k01_s01_a01_r01_depth_00000.pgm", 
      CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR 
  );

  sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
  sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth_image).toImageMsg();
  sensor_msgs::CameraInfo camera_info_msg;

  // read the camera intrinsics
  std::vector<float> camera_intrinsics;
  string camera_intrinsics_string;
  nh.getParam("camera_intrinsics", camera_intrinsics_string);
  {
    std::istringstream is(camera_intrinsics_string);
    double value;
    while (is >> value) {
      camera_intrinsics.push_back(value);
    }
  }

  if (camera_intrinsics.size() == 4) {

  } else {
    ROS_WARN("Camera intrinsics not set!");
        
  }

  

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    rgb_pub.publish(rgb_msg);
    depth_pub.publish(depth_msg);
    
    camera_info_msg.header.stamp = ros::Time::now();
    camera_info_pub.publish(camera_info_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}