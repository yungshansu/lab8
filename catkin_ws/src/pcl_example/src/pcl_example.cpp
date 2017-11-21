#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
	pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
  //Exmaple : pcl PointCloudXYZRGB information
  printf("-------------------------Cloud information-----------------------------\n");
  printf(":");
  printf("cloud size: %d\n",cloud->points.size());
  int cloud_size=cloud->points.size();
  printf("The first cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf, R: %d, G: %d, B: %d\n",cloud->points[0].x,cloud->points[0].y,cloud->points[0].z,cloud->points[0].r,cloud->points[0].g,cloud->points[0].b);
  printf("The last cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf, R: %d, G: %d, B: %d\n",cloud->points[cloud_size-1].x,cloud->points[cloud_size-1].y,cloud->points[cloud_size-1].z,cloud->points[cloud_size-1].r,cloud->points[cloud_size-1].g,cloud->points[cloud_size-1].b);
  printf("**********************************************************************\n");
  
  //Exercise : Calculate the location of the object (coordinate average)
  double x=0; double y=0;double z=0;
  ///Write your algorithm/////////////////
  for(int i=0;i<cloud->points.size();i++){
    x+=cloud->points[i].x;
    y+=cloud->points[i].y;
    z+=cloud->points[i].z;
  }
  x /=cloud->points.size();
  y /=cloud->points.size();
  z /=cloud->points.size();

  /////////////////////////////////////////


  printf("The model location:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf\n",x,y,z);


}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera_link/moving_object", 1, cloud_cb);

     // Spin
     ros::spin ();
  }
