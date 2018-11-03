#include "stdafx.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>//模型系数定义头文件
#include <pcl/filters/project_inliers.h> //投影滤波类头文件
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  


	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width*cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	//定义模型系数对象，并填充对应的数据
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 1;
	coefficients->values[1] = 0;
	coefficients->values[2] = 0;
	coefficients->values[3] = 0;

	pcl::ProjectInliers<pcl::PointXYZ> proj;      //创建投影滤波对象
	proj.setModelType(pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
	proj.setInputCloud(cloud);                    //设置输入点云
	proj.setModelCoefficients(coefficients);     //设置模型对应的系数
	proj.filter(*cloud_projected);                //执行投影滤波存储结果cloud_projected

	std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;
	system("pause");
	return 0;
}
