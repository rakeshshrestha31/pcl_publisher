#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

sensor_msgs::CameraInfo camera_info_global;
bool is_camera_info = false;

void pclSubscriber(const PointCloudT::ConstPtr& msg);

void cameraInfoSubscriber(sensor_msgs::CameraInfo msg)
{
  is_camera_info = true;
  camera_info_global = msg;
}

void extractPerson(PointCloudT::Ptr cloud);


void pclSubscriber(const PointCloudT::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // pcl::io::savePCDFileASCII ("/home/rakesh/person5.pcd", *msg);

  PointCloudT::Ptr cloud (new PointCloudT( *msg ) );
  extractPerson( cloud );
}

void extractPerson(PointCloudT::Ptr cloud)
{
  // Algorithm parameters:
  std::string svm_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/parameters/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;

  Eigen::VectorXf ground_coeffs(4);
  ground_coeffs.resize(4);
  ground_coeffs(0) = 0.0203208;
  ground_coeffs(1) = 0.988822;
  ground_coeffs(2) = 0.246924;
  ground_coeffs(3) = -2.03436;

  Eigen::Matrix3f rgb_intrinsics_matrix;
  
  if (is_camera_info) {
    rgb_intrinsics_matrix << camera_info_global.K[0], camera_info_global.K[1], camera_info_global.K[2], camera_info_global.K[3], 
      camera_info_global.K[4], camera_info_global.K[5], camera_info_global.K[6], camera_info_global.K[7], camera_info_global.K[8];
  }

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

  std::cout << "here" << std::endl;

  // Perform people detection on the new cloud:
  std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
  people_detector.setInputCloud(cloud);
  people_detector.setGround(ground_coeffs);                    // set floor coefficients
  people_detector.compute(clusters);                           // perform people detection

  ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

  PointCloudT::Ptr no_ground_cloud (new PointCloudT);
  no_ground_cloud = people_detector.getNoGroundCloud();

  unsigned int k = 0;

  PointCloudT::Ptr cluster_cloud (new PointCloudT);
  PointCloudT::Ptr transformed_cloud (new PointCloudT);
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
    {
      Eigen::Vector3f top = it->getTop();
      Eigen::Vector3f bottom = it->getBottom();

      Eigen::Vector4f minPoints;
      minPoints[0] = top[0]-0.5;
      minPoints[1] = top[1];
      minPoints[2] = top[2]-0.5;
      minPoints[3] = 1;

      Eigen::Vector4f maxPoints;
      maxPoints[0] = bottom[0]+0.5;
      maxPoints[1] = bottom[1];
      maxPoints[2] = bottom[2]+0.5;
      maxPoints[3] = 1;

      std::cout << maxPoints << std::endl << " " << std::endl << minPoints << std::endl;

      pcl::CropBox<PointT> cropFilter;
      cropFilter.setInputCloud(cloud);
      cropFilter.setMin(minPoints);
      cropFilter.setMax(maxPoints);
      cropFilter.filter(*cluster_cloud);

      Eigen::Affine3f transform_origin = Eigen::Affine3f::Identity();
      transform_origin.translation() << -minPoints[0], -minPoints[1], -minPoints[2];
      pcl::transformPointCloud(*cluster_cloud, *transformed_cloud, transform_origin);

      PointCloudT::Ptr voxel_filtered (new PointCloudT ());

      // Create the filtering object
      pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud (transformed_cloud);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*voxel_filtered);

      std::cerr << "PointCloud after filtering: " << voxel_filtered->width * voxel_filtered->height 
       << " data points (" << pcl::getFieldsList (*voxel_filtered) << ").";

//      pcl::PCDWriter writer;
//         writer.write ("table_scene_lms400_downsampled.pcd", *voxel_filtered, 
//         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

      pcl::io::savePCDFileASCII ("/home/rakesh/person.pcd", *voxel_filtered);

/*
      // draw theoretical person bounding box in the PCL viewer:
      pcl::PointIndices clusterIndices = it->getIndices();    // cluster indices
      std::vector<int> indices = clusterIndices.indices;
      for(unsigned int i = 0; i < indices.size(); i++)        // fill cluster cloud
      {
        PointT* p = &no_ground_cloud->points[indices[i]];
        cluster_cloud->push_back(*p);
      }
*/

      k++;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_extractor");
  ros::NodeHandle nh;

  ros::Subscriber sub_pcl = nh.subscribe<PointCloudT>("/depth_registered/points", 1, pclSubscriber);
  ros::Subscriber sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("/rgb/camera_info", 1, cameraInfoSubscriber);
  ros::spin();

}