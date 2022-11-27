#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#define SAFE_FREE(ptr) {free(ptr); ptr = NULL};
ros::Publisher pub;
tf::StampedTransform cam2_tf;
void transformationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ros::Rate rate(10.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInput (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::fromROSMsg(*input, *cloudInput);
  passFilter(cloudInput, cloudFiltered);
  voxelFilter (cloudFiltered, cloudFiltered2);
  transformationFilter(cloudFiltered2, transformed_cloud);
  statisticalFilter(transformed_cloud, cloudOut);

  // ---------------- rotation filter --------------//
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRotated (new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl_ros::transformPointCloud (*transformed_cloud, *cloudRotated, cam2_tf);
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&cloudRotated);
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

void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  if (input->size() <= 0) return;
  // ---------------- voxel filter --------------//
  pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
  voxelFilter.setInputCloud(input);
  voxelFilter.setLeafSize(0.1, 0.1, 0.1);
  voxelFilter.filter(*output);
}

void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  // ---------------- pass filter --------------//
  pcl::PassThrough<pcl::PointXYZ> passFilter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered3(new pcl::PointCloud<pcl::PointXYZ>);

  passFilter.setInputCloud(input);
  passFilter.setFilterFieldName("x");
  passFilter.setFilterLimits(-4.0, 4.0);
  // pass.setFilterLimitNegative(true) // false: keep, true: reject
  passFilter.filter(*cloudFiltered2);

  passFilter.setInputCloud(cloudFiltered2);
  passFilter.setFilterFieldName("y");
  passFilter.setFilterLimits(-0.4, 3.0);
  passFilter.filter(*cloudFiltered3);

  passFilter.setInputCloud(cloudFiltered3);
  passFilter.setFilterFieldName("z");
  passFilter.setFilterLimits(0.7, 1.2);
  passFilter.filter(*output);
}

void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::visualization::PCLVisualizer viewer("cloud");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "x");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample");
  viewer.addCoordinateSystem(0.1);
  viewer.initCameraParameters();
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::PointXYZ p1(centroid[0], centroid[1], centroid[2]);
  pcl::PointXYZ p2(centroid[0], centroid[1]+0.03, centroid[2]);
  viewer.addArrow(p2,p1,1,0,0,false);
  while(!viewer.wasStopped()){
    viewer.spinOnce();
  }
}

void transformationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
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

void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  if (input->size() <= 0) return;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> staticFilter;
  staticFilter.setInputCloud(input);
  staticFilter.setMeanK(5);
  staticFilter.setStddevMulThresh(0.5);
  staticFilter.filter(*output);
  return;
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
  // Spin
  ros::spin();
  return 0;
}
