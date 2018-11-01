#include "stdafx.h"
#include "iostream"
#include <ctime>
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

	//新建一个可视化类对象viewer
	pcl::visualization::PCLVisualizer viewer("cloud example");

	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 120, 80);
	viewer.addPointCloud(cloud, source_cloud_color_handler, "cloud example");

	//添加坐标系
	viewer.addCoordinateSystem(1000.0, 0);

	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer.initCameraParameters();

	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud example");

	//让可视化视窗停住，否则一闪而过。
	while (!viewer.wasStopped()) 
	{
		viewer.spinOnce();
	}
	
	return 0;
}
