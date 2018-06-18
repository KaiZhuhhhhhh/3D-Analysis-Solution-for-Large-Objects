#include "stdafx.h"
#include "pointCloudProcess.h"

pointCloudProcess pCP;

void loadPointCloud(const string fileName,
	pcl::PointCloud<PointT>::Ptr &cloud)
{
	//pcl::PCDReader reader;
	pcl::PLYReader reader;
	reader.read(fileName, *cloud); 

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	
	//test£¬ÏÔÊ¾
	//showPointCloud(cloud, "original poing cloud");
}

void downSample(pcl::PointCloud<PointT>::Ptr &cloud,
	pcl::PointCloud<PointT>::Ptr &cloud_downSampled)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(pCP.LeafSize, pCP.LeafSize, pCP.LeafSize);
	sor.filter(*cloud_downSampled);

	std::cerr << "PointCloud after filtering: " << cloud_downSampled->width * cloud_downSampled->height
		<< " data points (" << pcl::getFieldsList(*cloud_downSampled) << ").";

	//test£¬ÏÔÊ¾
	//showCompare2PointCloud(cloud, cloud_downSampled, "downsample");
//	showPointCloud(cloud_downSampled, "downsampled point cloud");
}

void filterPointCloud(pcl::PointCloud<PointT>::Ptr &original_cloud,
	pcl::PointCloud<PointT>::Ptr &cloud_filtered)
{

  pcl::StatisticalOutlierRemoval<PointT> sor;   
  sor.setInputCloud(original_cloud);                          
  sor.setMeanK(pCP.MeanK);
  sor.setStddevMulThresh(pCP.StddevMulThresh);
  sor.filter (*cloud_filtered);                   
  std::cout << "PointCloud after filtering: " << original_cloud->width * original_cloud->height
	  << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
//  system("PAUSE");
  
  //test,ÏÔÊ¾
  //showPointCloud(cloud_filtered, "filtered point cloud");
  //showCompare2PointCloud(original_cloud, cloud_filtered, "filtered");
}

void showPointCloud(pcl::PointCloud<PointT>::Ptr &cloud, string windowName)
{
	pcl::visualization::CloudViewer viewer(windowName);
	viewer.showCloud(cloud);
	system("PAUSE");
}

void showCompare2PointCloud(pcl::PointCloud<PointT>::Ptr &cloud1,
	pcl::PointCloud<PointT>::Ptr &cloud2, string windowName)
{
	//test 
	pcl::visualization::PCLVisualizer viewer(windowName);
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud1, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud1, cloud_in_color_h, "cloud_in_v1", v1);       //viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_out_green(cloud2, 20, 180, 20);
	viewer.addPointCloud(cloud2, cloud_out_green, "cloud_out", v2);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	viewer.setSize(1280, 1024);  // Visualiser window size
	//viewer.showCloud(cloud_out);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	system("PAUSE");
}