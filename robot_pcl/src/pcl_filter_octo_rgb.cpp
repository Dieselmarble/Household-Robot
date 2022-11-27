#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <octomap_server/OctomapServer.h>
#include <iostream>

#define SAFE_FREE(ptr) {free(ptr); ptr = NULL};
ros::Publisher pub;
tf::StampedTransform cam2_tf;
ros::ServiceClient octoClearClient;
void transformationFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud);
void visualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void passFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void passFilter2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void statisticalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void radiusFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
void timerCallback(const ros::TimerEvent& e);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ros::Rate rate(10.0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInput (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered3 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered4 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered5 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());


  pcl::fromROSMsg(*input, *cloudInput);
  voxelFilter (cloudInput, cloudFiltered2);
  transformationFilter(cloudFiltered2, cloudFiltered3);
  passFilter(cloudFiltered3, cloudFiltered4);
  // passFilter2(cloudFiltered3, cloudFiltered4);
  statisticalFilter(cloudFiltered4, cloudFiltered5);
  radiusFilter(cloudFiltered5, cloudOut);

  // ---------------- rotation filter --------------//
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRotated (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // pcl_ros::transformPointCloud (*transformed_cloud, *cloudRotated, cam2_tf);
  
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(&cloudRotated);
  //visualization(cloud);

  // pcl::PCLPointCloud2* cloudRotated2 = new pcl::PCLPointCloud2;
  // pcl::toPCLPointCloud2(*transformed_cloud, *cloudRotated2);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloudOut, output);
  // Publish the data
  pub.publish (output);
  rate.sleep();
}



void voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (input->size() <= 0) return;
  // ---------------- voxel filter --------------//
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
  voxelFilter.setInputCloud(input);
  voxelFilter.setLeafSize(0.05, 0.05, 0.05);
  voxelFilter.filter(*output);
}

void passFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  // ---------------- pass filter --------------//
  pcl::PassThrough<pcl::PointXYZRGB> passFilter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered3(new pcl::PointCloud<pcl::PointXYZRGB>);

  passFilter.setInputCloud(input);
  passFilter.setFilterFieldName("x");
  passFilter.setFilterLimits(-5.0, 5.0);//(-4,4)
  // pass.setFilterLimitNegative(true) // false: keep, true: reject
  passFilter.filter(*cloudFiltered2);

  passFilter.setInputCloud(cloudFiltered2);
  passFilter.setFilterFieldName("y");
  passFilter.setFilterLimits(-2.4, 0.4); //(-2.4, 2.4)
  passFilter.filter(*cloudFiltered3);

  passFilter.setInputCloud(cloudFiltered3);
  passFilter.setFilterFieldName("z");
  passFilter.setFilterLimits(-1.7, 0.7);//(0.7,3)
  passFilter.filter(*output);
}


void passFilter2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  // ---------------- pass filter --------------//
  pcl::PassThrough<pcl::PointXYZRGB> passFilter;

  passFilter.setInputCloud(input);
  passFilter.setFilterFieldName("z");
  passFilter.setFilterLimits(-0.4, 0.6);//(-0.9,0.4)
  // pass.setFilterLimitNegative(true) // false: keep, true: reject
  passFilter.filter(*output);
}


void visualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::visualization::PCLVisualizer viewer("cloud");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> fildColor(cloud, "x");
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, fildColor, "sample");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample");
  viewer.addCoordinateSystem(0.1);
  viewer.initCameraParameters();
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::PointXYZRGB p1(centroid[0], centroid[1], centroid[2]);
  pcl::PointXYZRGB p2(centroid[0], centroid[1]+0.03, centroid[2]);
  viewer.addArrow(p2,p1,1,0,0,false);
  while(!viewer.wasStopped()){
    viewer.spinOnce();
  }
}

void transformationFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (input->size() <= 0) return;
  float theta = M_PI/2; // The angle of rotation in radians
  float theta2 = 3*M_PI/2;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf (theta2, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX());
  transform.translation() << 0.0, 0.0, -0.1;
  // The same rotation matrix as before; theta radians around Z axis
  transform.rotate (q);
  transformPointCloud (*input, *output, transform);
  return;
}

void statisticalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (input->size() <= 0) return;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> staticFilter;
  staticFilter.setInputCloud(input);
  staticFilter.setMeanK(30);
  staticFilter.setStddevMulThresh(0.1);
  staticFilter.filter(*output);
  return;
}



void radiusFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (input->size() <= 0) return;
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusFilter;
  radiusFilter.setInputCloud(input);
  radiusFilter.setRadiusSearch(0.20);
  radiusFilter.setMinNeighborsInRadius(10);//10
  radiusFilter.setKeepOrganized(true);
  radiusFilter.filter(*output);
  return;
}
void octoClearCallback(const ros::TimerEvent& e)
{
  std_srvs::Empty srv;
  octoClearClient.call(srv);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "robot_pcl_node");
  ros::NodeHandle nh;

  // tf::TransformListener listener;
  // listener.waitForTransform("/base_link", "/cam_2_link", ros::Time(0), ros::Duration(10.0));
  // ROS_INFO("transform find\n");
  // try {
  //   listener.lookupTransform("/base_link", "/cam_2_link", ros::Time(0), cam2_tf);
  // } catch (tf::TransformException ex) {
  //   ROS_WARN("%s", ex.what());
  //   ros::Duration(0.5).sleep();
  // }
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cam_2/depth/color/points", 5, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cam_2/depth/color/points/filtered", 10);
  // octoClearClient = nh.serviceClient<std_srvs::Empty>("/octomap_server/reset");
  // octoClearClient.waitForExistence();
  // ros::Timer octoClearTimer = nh.createTimer(ros::Duration(2.0), octoClearCallback);
  ros::spin();
  return 0;
}
