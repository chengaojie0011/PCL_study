#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数
using namespace std;

int main(int argc, char** argv)
{

	/**********************************************************************************************************
	从输入的.PCD 文件载入数据后，创建一个VOxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程，
	越少的点意味着分割循环中处理起来越快
	**********************************************************************************************************/

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);//申明滤波前后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), 
		cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取PCD文件
	pcl::PCDReader reader;
	reader.read("table_scene_lms400.pcd", *cloud_blob);
	//统计滤波前的点云个数
	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// 创建体素栅格下采样: 下采样的大小为1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
	sor.setInputCloud(cloud_blob);             //原始点云
	sor.setLeafSize(0.01f, 0.01f, 0.01f);    // 设置采样体素大小
	sor.filter(*cloud_filtered_blob);        //保存

	// 转换为模板点云
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// 保存下采样后的点云
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<pcl::PointXYZ> seg;               //创建分割对象

	seg.setOptimizeCoefficients(true);                    //设置对估计模型参数进行优化处理

	seg.setModelType(pcl::SACMODEL_PLANE);                //设置分割模型类别
	seg.setMethodType(pcl::SAC_RANSAC);                   //设置用哪个随机参数估计方法
	seg.setMaxIterations(1000);                            //设置最大迭代次数
	seg.setDistanceThreshold(0.01);                      //判断是否为模型内点的距离阀值

	// 设置ExtractIndices的实际参数
	pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	/*	std::stringstream ss;
		ss << "table_scene_lms400_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);*/

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);



		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

		viewer->initCameraParameters();
		int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0, v1);
		viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1(cloud_p, 255, 120, 80);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_p, source_cloud_color_handler1, "sample cloud1", v1);

		int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
		viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2(cloud_f, 0, 0, 255);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_f, source_cloud_color_handler2, "sample cloud2", v2);

		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
		viewer->addCoordinateSystem(1.0);

		//让可视化视窗停住，否则一闪而过。
		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}


		break;


		i++;
	}

	return (0);
}