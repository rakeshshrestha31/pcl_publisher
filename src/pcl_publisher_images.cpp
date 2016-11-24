#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>


using namespace std;

void getVectorFromString(string str, std::vector<double> &vector)
{
  std::istringstream is(str);
  double value;
  while (is >> value) {
    vector.push_back(value);
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  string rgb_topic = "rgb/image_rect_color";
  string depth_topic = "depth_registered/image_rect";
  string camera_info_topic = "rgb/camera_info";
  // default camera params
  string camera_intrinsics_string = "531.49230957 532.39190674 314.63775635 252.53335571";
  string camera_extrinsics_R_string = "0.869593024 0.005134047 -0.493742496 0.083783410 -0.986979902 0.137298822 -0.486609042 -0.160761520 -0.858700991";
  string camera_extrinsics_t_string = "-844.523864746 763.838439941 3232.193359375";

    
  nh.getParam("/pcl_publisher/rgb_topic", rgb_topic);
  nh.getParam("/pcl_publisher/depth_topic", depth_topic);
  nh.getParam("/pcl_publisher/camera_info_topic", camera_info_topic);
  nh.getParam("/pcl_publisher/camera_intrinsics", camera_intrinsics_string);
  nh.getParam("/pcl_publisher/camera_extrinsics_R", camera_extrinsics_R_string);
  nh.getParam("/pcl_publisher/camera_extrinsics_t", camera_extrinsics_t_string);

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
  std::vector<double> camera_intrinsics, camera_extrinsics_R, camera_extrinsics_t;

  getVectorFromString(camera_intrinsics_string, camera_intrinsics);
  getVectorFromString(camera_extrinsics_R_string, camera_extrinsics_R);
  getVectorFromString(camera_extrinsics_t_string, camera_extrinsics_t);
  
  if (camera_intrinsics.size() == 4) {
    camera_info_msg.K[0] = camera_intrinsics.at(0);
    camera_info_msg.K[4] = camera_intrinsics.at(1);
    camera_info_msg.K[2] = camera_intrinsics.at(2);
    camera_info_msg.K[5] = camera_intrinsics.at(3);
    camera_info_msg.K[8] = 1;

    camera_info_msg.P[0] = camera_intrinsics.at(0);
    camera_info_msg.P[2] = camera_intrinsics.at(1);
    camera_info_msg.P[5] = camera_intrinsics.at(2);
    camera_info_msg.P[6] = camera_intrinsics.at(3);
    camera_info_msg.P[10] = 1;
    
  } else {
    ROS_WARN("Camera intrinsics not set!");
  }

  if (camera_extrinsics_R.size() == 9) {
    memcpy(&camera_info_msg.R[0], &camera_extrinsics_R[0], 9 * sizeof(double));
  } else {
    ROS_WARN("Camera extrinsic (R) not set!");
  }

  if (camera_extrinsics_t.size() == 3) {
    camera_info_msg.P[3] = camera_extrinsics_t.at(0);
    camera_info_msg.P[7] = camera_extrinsics_t.at(1);
  } else {
    ROS_WARN("Camera extrinsic (t) not set!");
  }



  

  camera_info_msg.width = rgb_image.cols;
  camera_info_msg.height = rgb_image.rows;
  

  // new code: faraz
  camera_info_msg.height = rgb_image.rows;
  camera_info_msg.width = rgb_image.cols;
  camera_info_msg.distortion_model = "plumb_bob";
  
  camera_info_msg.D.push_back(0.19607373);
  camera_info_msg.D.push_back(-0.36734107);
  camera_info_msg.D.push_back(-2.47962005e-003);
  camera_info_msg.D.push_back(-1.89774996e-003);

  // camera_info_msg.D[0] = 0.19607373;
  // camera_info_msg.D[1] = -0.36734107;
  // camera_info_msg.D[2] = -2.47962005e-003;
  // camera_info_msg.D[3] = -1.89774996e-003;
  
  camera_info_msg.K[0] = 531.49230957;
  camera_info_msg.K[1] = 0.;
  camera_info_msg.K[2] = 314.63775635;
  camera_info_msg.K[3] = 0.;
  camera_info_msg.K[4] = 532.39190674;
  camera_info_msg.K[5] = 252.53335571;
  camera_info_msg.K[6] = 0.;
  camera_info_msg.K[7] = 0.;
  camera_info_msg.K[8] = 1.;

  camera_info_msg.R[0] = 0.869593024;
  camera_info_msg.R[1] = 0.005134047;
  camera_info_msg.R[2] = -0.493742496;
  camera_info_msg.R[4] = 0.083783410;
  camera_info_msg.R[3] = -0.986979902;
  camera_info_msg.R[5] = 0.137298822;
  camera_info_msg.R[6] = -0.486609042;
  camera_info_msg.R[7] = -0.160761520;
  camera_info_msg.R[8] = -0.858700991;

  camera_info_msg.P[0] = 531.49230957;
  camera_info_msg.P[1] = 0.;
  camera_info_msg.P[2] = 532.39190674;
  camera_info_msg.P[3] = -844.523864746;
  camera_info_msg.P[4] = 0.;
  camera_info_msg.P[5] = 314.63775635;
  camera_info_msg.P[6] = 252.53335571;
  camera_info_msg.P[7] = 763.838439941;
  camera_info_msg.P[8] = 0.;
  camera_info_msg.P[9] = 0.;
  camera_info_msg.P[10] = 1.;
  camera_info_msg.P[11] = 0.;



  // end new code: faraz

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