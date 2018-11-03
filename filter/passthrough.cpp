#include "stdafx.h"
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main()
{
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
	// 生成并填充点云数据
	cloud->width = 1000;                                     //设置点云宽度或数量，这里为数量
	cloud->height = 1;                                    //设置点云高度或标准其为无序点云
	cloud->points.resize(cloud->width*cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i) //为点云填充数据
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//std::cerr << "Cloud before filtering: " << std::endl; //打印所有点到标准错误输出
	//for (size_t i = 0; i < cloud->points.size(); ++i)
	//	std::cerr << "    " << cloud->points[i].x << " "
	//	<< cloud->points[i].y << " "
	//	<< cloud->points[i].z << std::endl;

	pcl::PassThrough<pcl::PointXYZ>pass;     //设置滤波器对象
	pass.setInputCloud(cloud);                //设置输入点云
	pass.setFilterFieldName("z");             //设置过滤时所需要点云类型的z字段
	pass.setFilterLimits(0.0,500.0);           //设置在过滤字段上的范围
	//pass.setFilterLimitsNegative (true);     //设置保留范围内的还是过滤掉范围内的
	pass.filter(*cloud_filtered);              //执行滤波，保存过滤结果在cloud_filtered


	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;

	//新建一个可视化类对象viewer
	pcl::visualization::PCLVisualizer viewer("cloud example");

	//设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud_filtered, 255, 120, 80);
	viewer.addPointCloud(cloud_filtered, source_cloud_color_handler, "cloud example");

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



	system("pause");
	return 0;
}
