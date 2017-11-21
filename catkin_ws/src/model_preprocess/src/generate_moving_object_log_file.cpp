#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <image_transport/image_transport.h>
#include <math.h>
//using namespace cv;
using namespace std;
using namespace pcl;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher cloud_object_publisher;


void  generate_move_object (PointCloudXYZRGB::Ptr cloud){
	PointCloudXYZRGB::Ptr cloud_shift (new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud,*cloud_shift);	//Copy pointcloud
	printf("--------------------------\n");
	printf("cloud size: %d\n",cloud_shift->points.size());
	//Shift point cloud and change its color to observe convenienctly
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	PointCloudXYZRGB::Ptr transformed_cloud (new PointCloudXYZRGB());
	PointCloudXYZRGB::Ptr transformed_cloud_reg (new PointCloudXYZRGB());
	transformed_cloud ->header.frame_id = "/camera_link";
	for (double length =1.0/16; length <1.0; length+= 1.0/16 ){
	  	transform_1 (0,3) = length;
	  	pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	for(int a=0;a<200000000;a++){}
	}
	transform_1 = Eigen::Matrix4f::Identity();
	copyPointCloud(*cloud,*transformed_cloud_reg);
	for (double angle =1.0/16; angle <=1.0/2.0; angle+= 1.0/16 ){
		float theta = M_PI*angle; // The angle of rotation in radians
		transform_1 (0,3) = 1.0;
	  	transform_1 (0,0) = cos (theta);
	  	transform_1 (0,1) = -sin(theta);
	  	transform_1 (1,0) = sin (theta);
	  	transform_1 (1,1) = cos (theta);
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	printf("%lf angle\n",theta);
	  	for(int a=0;a<200000000;a++){}
	}
	transform_1 = Eigen::Matrix4f::Identity();
	copyPointCloud(*transformed_cloud,*transformed_cloud_reg);
	for (double length =1.0/16; length <1.0; length+= 1.0/16 ){
	  	transform_1 (1,3) = length;
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	for(int a=0;a<200000000;a++){}
	}
transform_1 = Eigen::Matrix4f::Identity();
	copyPointCloud(*cloud,*transformed_cloud_reg);
	for (double angle =1.0/16; angle <=1.0/2.0; angle+= 1.0/16 ){
		float theta = M_PI*angle; // The angle of rotation in radians
		transform_1 (0,3) = 1.0;
		transform_1 (1,3) = 1.0;
	  	transform_1 (0,0) = cos (M_PI/2.0+theta);
	  	transform_1 (0,1) = -sin(M_PI/2.0+theta);
	  	transform_1 (1,0) = sin (M_PI/2.0+theta);
	  	transform_1 (1,1) = cos (M_PI/2.0+theta);
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	printf("%lf angle\n",theta);
	  	for(int a=0;a<200000000;a++){}
	}
transform_1 = Eigen::Matrix4f::Identity();
	copyPointCloud(*transformed_cloud,*transformed_cloud_reg);
	for (double length =1.0/16; length <1.0; length+= 1.0/16 ){
	  	transform_1 (0,3) = -length;
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	for(int a=0;a<200000000;a++){}
	}
transform_1 = Eigen::Matrix4f::Identity();
copyPointCloud(*cloud,*transformed_cloud_reg);
	for (double angle =1.0/16; angle <=1.0/2.0; angle+= 1.0/16 ){
		float theta = M_PI*angle; // The angle of rotation in radians
		transform_1 (1,3) = 1.0;
	  	transform_1 (0,0) = cos (M_PI+theta);
	  	transform_1 (0,1) = -sin(M_PI+theta);
	  	transform_1 (1,0) = sin (M_PI+theta);
	  	transform_1 (1,1) = cos (M_PI+theta);
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	printf("%lf angle\n",theta);
	  	for(int a=0;a<200000000;a++){}
	}
transform_1 = Eigen::Matrix4f::Identity();
	copyPointCloud(*transformed_cloud,*transformed_cloud_reg);
	for (double length =1.0/16; length <1.0; length+= 1.0/16 ){
	  	transform_1 (1,3) = -length;
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	for(int a=0;a<200000000;a++){}
	}
transform_1 = Eigen::Matrix4f::Identity();
copyPointCloud(*cloud,*transformed_cloud_reg);
	for (double angle =1.0/16; angle <=1.0/2.0; angle+= 1.0/16 ){
		float theta = M_PI*angle; // The angle of rotation in radians
		transform_1 (1,3) = 0.0;
	  	transform_1 (0,0) = cos (M_PI*1.5+theta);
	  	transform_1 (0,1) = -sin(M_PI*1.5+theta);
	  	transform_1 (1,0) = sin (M_PI*1.5+theta);
	  	transform_1 (1,1) = cos (M_PI*1.5+theta);
	  	pcl::transformPointCloud (*transformed_cloud_reg, *transformed_cloud, transform_1);
	  	sensor_msgs::PointCloud2 output;
	  	pcl::toROSMsg(*transformed_cloud, output);
	  	output.header.frame_id="/camera_link";
	  	cloud_object_publisher.publish( output);
	  	printf("%lf angle\n",theta);
	  	for(int a=0;a<200000000;a++){}
	}

	
	return;
}



int   main (int argc, char** argv)
{
	// Declare ROS publisher 
    ros::init (argc, argv, "ICP_example1");
    ros::NodeHandle nh("~");  
    cloud_object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera_link/moving_object", 1);
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
 		PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
 		generate_move_object(cloud_model);
    }
}
