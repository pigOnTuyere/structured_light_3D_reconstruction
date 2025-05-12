#pragma once
#include<iostream>
#include<ctime>
#include<vector>
#include <pcl/common/io.h>
//#include<pcl/io/io.h>
//#include<pcl/io/pcd_io.h>
//#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>

class MyPointCloud
{
public:
	MyPointCloud();
	~MyPointCloud();

	void testPrint() {
		std::cout << "MyPointCloud testPrint" << std::endl;
	}

	void showSrcCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr2);
	void showSrcCloud_TextureMapping(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_ptr);
};


