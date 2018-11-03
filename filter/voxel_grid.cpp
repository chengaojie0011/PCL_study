#include "stdafx.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1  
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。  
		return (-1);
	}

	pcl::VoxelGrid<pcl::PointXYZ>sor;  //创建滤波对象
	sor.setInputCloud(cloud);                       //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.01f, 0.01f, 0.01f);           //设置滤波时创建的体素大小为1cm立方体
	sor.filter(*cloud_filtered);                   //执行滤波处理，存储输出cloud_filtered

	cerr << "cloud->points.size():"<< cloud->points.size() << endl;
	cerr << "cloud->points.size():" << cloud_filtered->points.size() << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->initCameraParameters();
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1(cloud, 255, 120, 80);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, source_cloud_color_handler1,"sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2(cloud_filtered, 0,0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, source_cloud_color_handler2, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	//让可视化视窗停住，否则一闪而过。
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}



	system("pause");
	return 0;
}
