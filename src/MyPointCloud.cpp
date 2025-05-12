#include "MyPointCloud.h"

#include <memory>
#include <thread>

MyPointCloud::MyPointCloud()
{
}


MyPointCloud::~MyPointCloud()
{
}

void MyPointCloud::showSrcCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr2)
{

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>rgb(src_cloud_ptr, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(src_cloud_ptr, rgb, "cloud");

	// 对源点云着色可视化 (blue).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(src_cloud_ptr2, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(src_cloud_ptr2, input_color, "input cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(1000));
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}

}


void MyPointCloud::showSrcCloud_TextureMapping(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_ptr)
{

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>rgb(src_cloud_ptr, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(src_cloud_ptr, "cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(1000));
		std::this_thread::sleep_for(std::chrono::microseconds(1000));

	}
}