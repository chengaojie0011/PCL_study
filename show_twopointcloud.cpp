#include "stdafx.h"
#include "iostream"
#include <ctime>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/io/ply_io.h> //PCL的PLY格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
#include <boost/thread/thread.hpp>
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

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->initCameraParameters();
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);





	// 添加直线
	//viewer.addLine(pcl::PointXYZ(0, 0, 512), pcl::PointXYZ(512, 512, 512));

	//让可视化视窗停住，否则一闪而过。
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	return 0;
}
