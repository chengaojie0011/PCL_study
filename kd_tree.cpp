#include "stdafx.h"
#include "iostream"

//#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main()
{
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  

	//初始化点云数据PCD文件头
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);          // c++中rand()函数生成的范围:0~RAND_MAX，
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}


	// 初始化kdTree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// 设置搜索点
	kdtree.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);


	// K近邻搜索

	int K = 10;

	std::vector<int> pointIdxNKNSearch(K);			//存储查询点近邻索引 
	std::vector<float> pointNKNSquaredDistance(K);	//／存储近邻点对应距离平方 

	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// 以radius为半径的范围搜索

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
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
