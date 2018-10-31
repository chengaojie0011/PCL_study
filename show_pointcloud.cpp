/**************************************************************************
**
**PointRegistration  Debug 1.0
**Copyright(c) 2017 - 2027 by *** Co.
**Written by Mr.Vincent
**
**Function:
**PointCloud All
**
**************************************************************************/
#include "stdafx.h"
#include "iostream"
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/io/ply_io.h> //PCL的PLY格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  

	if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\Program Files\\MATLAB\\R2018a\\toolbox\\vision\\visiondata\\teapot.ply", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1  
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。  
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_file.pcd with the following fields: "
		<< std::endl;

	pcl::visualization::PCLVisualizer viewer("cloud example");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 120, 80);
	viewer.addPointCloud(cloud, source_cloud_color_handler, "cloud example");
	//viewer.addCoordinateSystem(1.0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud example");
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	system("pause");
	return 0;
}
