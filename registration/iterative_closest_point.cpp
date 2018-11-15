#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>		//点类型定义头文件
#include <pcl/registration/icp.h>   //ICP配准类相关的头文件

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
		<< std::endl;

	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		std::cout << "    " <<cloud_in->points[i].x << " " 
		<< cloud_in->points[i].y << " " <<cloud_in->points[i].z << std::endl;

	*cloud_out = *cloud_in;

	std::cout << "size:" << cloud_out->points.size() << std::endl;

	//实现一个简单的点云刚体变换，以构造目标点云
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
		<< std::endl;

	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);													  //执行配准存储变换后的源点云到Final
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;								 //打印配准相关输入信息
	std::cout << icp.getFinalTransformation() << std::endl;				//打印输出最终估计的变换矩阵
	system("pause");
	return (0);
}