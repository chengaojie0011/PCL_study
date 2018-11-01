#include "stdafx.h"
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
#include <pcl/octree/octree.h>
using namespace std;

int main()
{
	srand(time(NULL));
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  

	//八叉树分辨率即体素的大小
	float resolution = 32.0f;
	//初始化空间变化检测对象
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

	//为cloudA点云填充点数据
	cloudA->width = 128;                                       //设置点云cloudA点数
	cloudA->height = 1;                                        //设置点云cloudA为无序点云
	cloudA->points.resize(cloudA->width *cloudA->height);

	for (size_t i = 0; i < cloudA->points.size(); ++i)     //循环产生点数据
	{
		cloudA->points[i].x = 64.0f* rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].y = 64.0f* rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].z = 64.0f* rand() / (RAND_MAX + 1.0f);
	}

	//添加点云到八叉树，构建八叉树
	octree.setInputCloud(cloudA);                            //设置输入点云
	octree.addPointsFromInputCloud();                       //从输入点云构建八叉树
	octree.switchBuffers();// 交换八叉树缓存，但是cloudA对应的八叉树结构仍在内存中

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

	// 为cloudB创建点云
	cloudB->width = 128;
	cloudB->height = 1;
	cloudB->points.resize(cloudB->width *cloudB->height);
	for (size_t i = 0; i < cloudB->points.size(); ++i)
	{
		cloudB->points[i].x = 64.0f* rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].y = 64.0f* rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].z = 64.0f* rand() / (RAND_MAX + 1.0f);
	}

	//添加 cloudB到八叉树
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();
		
	std::vector<int> newPointIdxVector;                        //存储新加点的索引的向量	
	//获取前一cloudA对应的八叉树在cloudB对应八叉树中没有的点集
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	//打印结果点到标准输出
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size(); ++i)      //循环打印所有结果点坐标
		std::cout << i << "# Index:" << newPointIdxVector[i]
		<< "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
		<< cloudB->points[newPointIdxVector[i]].y << " "
		<< cloudB->points[newPointIdxVector[i]].z << std::endl;

	pcl::PointCloud<pcl::RGB>::Ptr pPointsRGB(new pcl::PointCloud<pcl::RGB>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloudShow(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// 给搜索到的点上色，原始点云中的点全为蓝色，搜索到的上为红色
	pPointsRGB->width = cloudB->size();
	pPointsRGB->height = 1;
	pPointsRGB->resize(pPointsRGB->width*pPointsRGB->height);


	pCloudShow->width = cloudB->size();
	pCloudShow->height = 1;
	pCloudShow->resize(pCloudShow->width*pCloudShow->height);

	for (int i = 0; i < pPointsRGB->size(); i++)
		pPointsRGB->points[i].b = 255;

	for each(auto detected in newPointIdxVector)
		pPointsRGB->points[detected].r = 255;

	pcl::concatenateFields(*cloudB, *pPointsRGB, *pCloudShow);


	for (size_t i = 0; i < pPointsRGB->size(); i++)
	{
		pCloudShow->points[i].a = 255;

	}

	//新建一个可视化类对象viewer
	pcl::visualization::PCLVisualizer viewer("cloud example");

	//设置点云颜色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloudA, 255, 120, 80);
	//viewer.addPointCloud(cloudA, source_cloud_color_handler, "cloud example");
	viewer.addPointCloud(pCloudShow, "cloud example");
	//添加坐标系
	viewer.addCoordinateSystem(1000.0, 0);

	//通过设置照相机参数使得从默认的角度和方向观察点云
	//viewer.initCameraParameters();

	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer.setBackgroundColor(0.00, 0.05, 0.00, 0);

	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud example");

	//让可视化视窗停住，否则一闪而过。
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}
