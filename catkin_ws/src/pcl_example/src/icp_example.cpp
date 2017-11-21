#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace pcl;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher cloud_model_publisher;
ros::Publisher cloud_object_publisher;
ros::Publisher pose_publisher;
ros::Publisher cloud_icp_align_object_publisher;

void  task2 (PointCloudXYZRGB::Ptr cloud){
	PointCloudXYZRGB::Ptr cloud_shift (new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud,*cloud_shift);	//Copy pointcloud
	printf("--------------------------\n");
	printf("cloud size: %d\n",cloud_shift->points.size());
	//Shift point cloud and change its color to observe convenienctly
	for (int i=0;i<cloud_shift->points.size();i++){
		cloud_shift->points[i].r = 0;
		cloud_shift->points[i].x += 0.5;
		cloud_shift->points[i].y += 0.7;
		cloud_shift->points[i].z += 0.5;
	}
	
	////////////////Use PCL function to implement ICP  (line 53~67)///////////////////////////////////////////////
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;											//
	PointCloudXYZRGB::Ptr cloud_reg (new PointCloudXYZRGB);														//
	// Use kd tree to accelerate																				//
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);				//
	tree1->setInputCloud(cloud);																				//
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);				//
	tree2->setInputCloud(cloud_shift);																			//
	icp.setSearchMethodSource(tree1);																			//
	icp.setSearchMethodTarget(tree2);																			//
	icp.setInputSource(cloud);				// Set align model													//		
	icp.setInputTarget(cloud_shift);		// Set align target													//
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)	//												//
	icp.setMaxCorrespondenceDistance(1500);																		//
	icp.setTransformationEpsilon(1e-10);	// Set the transformation epsilon (criterion 1)						//										//
	icp.setEuclideanFitnessEpsilon(0.1);	// Set the euclidean distance difference epsilon (criterion 2)																	//
	icp.setMaximumIterations(300);			// Set the maximum number of iterations (criterion 3)				//													//
	icp.align(*cloud_reg);																						//
	Eigen::Matrix4f transformation = icp.getFinalTransformation();												//
	std::cout << icp.getFinalTransformation() << std::endl;														//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Print pose estimation outcome
	cloud_object_publisher.publish(*cloud_shift);
	cloud_icp_align_object_publisher.publish(*cloud_reg);
	return;
}


int   main (int argc, char** argv)
{
	// Declare ROS publisher 
    ros::init (argc, argv, "ICP_example1");
    ros::NodeHandle nh("~");  
	cloud_model_publisher = nh.advertise<PointCloudXYZRGB> ("/camera_link/pointcloudXYZ_model", 1);
    cloud_object_publisher = nh.advertise<PointCloudXYZRGB> ("/camera_link/pointcloudXYZ_object", 1);
    cloud_icp_align_object_publisher = nh.advertise<PointCloudXYZRGB> ("/camera_link/icp_align_model", 1);
    pose_publisher =  nh.advertise<nav_msgs::Path> ("/camera_link/pose_estimation", 1);

    //Load 3D PLY model
    std::string modelpath("dvd.ply");
    PointCloudXYZRGB::Ptr cloud_model (new PointCloudXYZRGB);
	ros::param::get("modelpath", modelpath);
	if (nh.getParam("modelpath",modelpath) ){
 		ROS_INFO("Got param: %s",modelpath.c_str());
	}
	printf("Model name =%s\n",modelpath.c_str() );
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (modelpath, *cloud_model) == -1){ // load 3D model to "cloud_model"
		PCL_ERROR ("Couldn't read ply file\n");
    	return (-1);

  	}
  	
  	//In this loop
    ros::Rate loop_rate(10);
    ROS_INFO("Ros Start");
    while (ros::ok())  {
    	cloud_model->header.frame_id = "/camera_link";
 		cloud_model_publisher.publish(*cloud_model);
 		PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
 		task2(cloud_model);
    }
}
