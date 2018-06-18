#pragma once
#define _CRT_SECURE_NO_WARNINGS  
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
using namespace std;
typedef pcl::PointXYZRGB PointT;
struct pointCloudProcess
{
	float LeafSize = 1.5;
	int MeanK = 80;
	float StddevMulThresh = 0.2;
	bool downsample_flag = true;
	bool filter_flag = true;
};
extern pointCloudProcess pCP;

void loadPointCloud(const string fileName,
		pcl::PointCloud<PointT>::Ptr &cloud);

void downSample(pcl::PointCloud<PointT>::Ptr &cloud,
		pcl::PointCloud<PointT>::Ptr &cloud_downSampled);

void filterPointCloud(pcl::PointCloud<PointT>::Ptr &original_cloud,
		pcl::PointCloud<PointT>::Ptr &cloud_filtered);

void showPointCloud(pcl::PointCloud<PointT>::Ptr &cloud, string windowName);

void showCompare2PointCloud(pcl::PointCloud<PointT>::Ptr &cloud1,
		pcl::PointCloud<PointT>::Ptr &cloud2, string windowName);






