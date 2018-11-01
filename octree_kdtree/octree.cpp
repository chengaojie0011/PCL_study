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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  

	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);          // c++中rand()函数生成的范围:0~RAND_MAX，
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}



	float resolution = 1024.0f;											//确定分辨率，即最小体素的边长
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); //初始化octree
	octree.setInputCloud(cloud);                                               //设置输入点云
	octree.addPointsFromInputCloud();                                         //构建octree

	pcl::PointXYZ searchPoint;

	searchPoint.x = 256.0f ;
	searchPoint.y =256.0f ;
	searchPoint.z =256.0f ;

	//searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	//searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	std::vector<int> pointIdxVec;                        //存储体素近邻搜索的结果向量
	if (octree.voxelSearch(searchPoint, pointIdxVec))    //执行搜索，返回结果到pointIdxVec
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z << ")"
			<< std::endl;

		std::cout << "Leaf Count : " << octree.getLeafCount() << std::endl;             // 叶子数
		std::cout << "Tree Depth : " << octree.getTreeDepth() << std::endl;             // 八叉树深度
		std::cout << "Branch Count : " << octree.getBranchCount() << std::endl;         // 非叶子结点数
		std::cout << "Voxel Diameter : " << octree.getVoxelSquaredDiameter() << std::endl;  // Voxel Side Length*3
		std::cout << "Voxel Side Length : " << octree.getVoxelSquaredSideLen() << std::endl;// 分辨率的平方
		double minx, miny, minz, maxx, maxy, maxz;
		octree.getBoundingBox(minx, miny, minz, maxx, maxy, maxz);
		std::cout << "BoundingBox: " << "(" << minx << " - " << maxx << ")" << " , "
			<< "(" << miny << " - " << maxy << ")" << " , " << "(" << minz << " - " << maxz << ")" << std::endl;          // 整个八叉树的范围

		for (size_t i = 0; i < pointIdxVec.size(); ++i)             //打印搜索结果点坐标
			std::cout << "  " << cloud->points[pointIdxVec[i]].x
			<< " " << cloud->points[pointIdxVec[i]].y
			<< " " << cloud->points[pointIdxVec[i]].z << std::endl;
	}


	////K近邻搜索

	//int K = 10;
	//std::vector<int> pointIdxNKNSearch;                           //存储k近邻搜索点索引结果
	//std::vector<float> pointNKNSquaredDistance;                  //与上面对应的平方距离
	//std::cout << "K nearest neighbor search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with K=" << K << std::endl;

	//if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	//{
	//	for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)           //打印搜索结果点坐标
	//		std::cout << "  2  " << cloud->points[pointIdxNKNSearch[i]].x
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].y
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].z
	//		<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	//}
	//

	////半径内近邻搜索
	//std::vector<int> pointIdxRadiusSearch;
	//std::vector<float> pointRadiusSquaredDistance;
	//float radius = 256.0f* rand() / (RAND_MAX + 1.0f);
	//std::cout << "Neighbors within radius search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with radius=" << radius << std::endl;
	//if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	//{
	//	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	//		std::cout << " 3  " << cloud->points[pointIdxRadiusSearch[i]].x
	//		<< " " << cloud->points[pointIdxRadiusSearch[i]].y
	//		<< " " << cloud->points[pointIdxRadiusSearch[i]].z
	//		<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

	//}


	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_file.pcd with the following fields: "
		<< std::endl;

	pcl::PointCloud<pcl::RGB>::Ptr pPointsRGB(new pcl::PointCloud<pcl::RGB>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloudShow(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// 给搜索到的点上色，原始点云中的点全为蓝色，搜索到的上为红色
	pPointsRGB->width = cloud->size();
	pPointsRGB->height = 1;
	pPointsRGB->resize(pPointsRGB->width*pPointsRGB->height);


	pCloudShow->width = cloud->size();
	pCloudShow->height = 1;
	pCloudShow->resize(pCloudShow->width*pCloudShow->height);

	for (size_t i = 0; i < pPointsRGB->size(); i++)
	{
		pPointsRGB->points[i].b = 255;
		pPointsRGB->points[i].g = 0;
		pPointsRGB->points[i].r = 0;
		pCloudShow->points[i].a = 255;
		//std::cout << " 3  " << cloud->points[i].x
		//	<< " " << cloud->points[i].y
		//	<< " " << cloud->points[i].z << std::endl;


	}

	for (size_t i = 0; i < pointIdxVec.size(); ++i)
	{
		pPointsRGB->points[pointIdxVec[i]].b = 0;
		pPointsRGB->points[pointIdxVec[i]].r = 255;
	}

	// 合并不同字段
	pcl::concatenateFields(*cloud, *pPointsRGB, *pCloudShow);


	for (size_t i = 0; i < pPointsRGB->size(); i++)
	{
		pCloudShow->points[i].a = 255;

	}



	//新建一个可视化类对象viewer
	pcl::visualization::PCLVisualizer viewer("cloud example");

	//设置点云颜色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 120, 80);
	//viewer.addPointCloud(cloud, source_cloud_color_handler, "cloud example");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> source_cloud_color_handler(pCloudShow, 255, 120, 80);
	viewer.addPointCloud(pCloudShow,  "cloud example");
	//添加坐标系
	viewer.addCoordinateSystem(1000.0, 0);

	//通过设置照相机参数使得从默认的角度和方向观察点云
	//viewer.initCameraParameters();

	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer.setBackgroundColor(0.05, 0.05, 0.00, 0);

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
