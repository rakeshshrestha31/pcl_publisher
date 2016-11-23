#include <iostream>
 
// Point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
 
// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// boost
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
 }

 boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
  

int main( int argc, char *argv[] ) {
    
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("/camera/depth_registered/points", 1);


    PointCloud::Ptr msg (new PointCloud);
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    
    cv::Mat depth_image = cv::imread(
        "/home/rakesh/rakesh/cmpt726/BerkeleyMHAD/Kinect/Kin01/S01/A01/R01/kin_k01_s01_a01_r01_depth_00000.pgm", 
        CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR 
    );

    cv::Mat rgb_image = cv::imread(
        "/home/rakesh/rakesh/cmpt726/BerkeleyMHAD/Kinect/Kin01/S01/A01/R01/kin_k01_s01_a01_r01_color_00000.ppm", 
        CV_LOAD_IMAGE_COLOR
    );

    depth_image.convertTo( depth_image, CV_32F); // convert the image data to float type
    // rgb_image.convertTo( rgb_image, CV_32F); // convert the image data to float type

    if(!depth_image.data){
        std::cerr << "No depth data!!!" << std::endl;
        exit(EXIT_FAILURE);
    }

    if(!rgb_image.data){
        std::cerr << "No RGB data!!!" << std::endl;
        exit(EXIT_FAILURE);
    }

    const float depth_factor = 5000; // mapping from png depth value to metric scale
    const float fx_d = 531.49230957f;
    const float fy_d = 532.39190674f;
    const float px_d = 314.63775635f;
    const float py_d = 252.53335571f;

    cloud.width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing
    cloud.height = depth_image.rows;
    cloud.resize(cloud.width*cloud.height);

    rgb_cloud.width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing
    rgb_cloud.height = depth_image.rows;
    rgb_cloud.resize(rgb_cloud.width*rgb_cloud.height);

    msg->header.frame_id = "/map";
    msg->height = depth_image.cols;
    msg->width = depth_image.rows;
    

    uint8_t* rgb_data = (uint8_t*)rgb_image.data;

    for(int v=0; v < depth_image.rows; v++) {
        //2-D indexing
        for(int u = 0; u < depth_image.cols; u++) {

            float z = depth_image.at<float>(v,u) / depth_factor;
            float x = z*(u-px_d)/fx_d;
            float y = z*(v-py_d)/fy_d;
            
            cloud(u,v).x = x;
            cloud(u,v).y = y;
            cloud(u,v).z = z;

            rgb_cloud(u,v).x = x;
            rgb_cloud(u,v).y = y;
            rgb_cloud(u,v).z = z;

            rgb_cloud(u,v).b = rgb_data[v * rgb_image.cols * 3 + u * 3 + 0];
            rgb_cloud(u,v).g = rgb_data[v * rgb_image.cols * 3 + u * 3 + 1];
            rgb_cloud(u,v).r = rgb_data[v * rgb_image.cols * 3 + u * 3 + 2];

            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            
            point.b = rgb_data[v * rgb_image.cols * 3 + u * 3 + 0];
            point.g = rgb_data[v * rgb_image.cols * 3 + u * 3 + 1];
            point.r = rgb_data[v * rgb_image.cols * 3 + u * 3 + 2];
            
            msg->points.push_back(point);

        } 
    }

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
      msg->header.stamp = ros::Time::now().toNSec();
      pub.publish (msg);
      ros::spinOnce ();
      loop_rate.sleep ();
    }
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr( &cloud );
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis( cloud_ptr );

    // pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ptr( &rgb_cloud );
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis( cloud_ptr );

    

    // while (!viewer->wasStopped ()) {
    //     viewer->spinOnce (100);
    //     boost::this_thread::sleep (boost::posiz_time::microseconds (100000));
    // }

    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud (cloud);

}

