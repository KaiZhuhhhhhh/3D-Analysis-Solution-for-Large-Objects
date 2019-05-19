#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/harris_3d.h>
#include <iostream>
#include <fstream>
typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_ = "Peach_align.ply";
std::string scene_filename_ = "8.5test2.ply";

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(true);
bool use_cloud_resolution_(false);
bool use_hough_(true);

//GSU调参
float model_ss_(6.0f);
float scene_ss_(6.0f);
float rf_rad_(8.0f);
float descr_rad_(8.0f);
float cg_size_(5.0f);
float cg_thresh_(1.2f);

ofstream outfile("camera.txt");
//隐藏代码
void
showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
	std::cout << "     -k:                     Show used keypoints." << std::endl;
	std::cout << "     -c:                     Show used correspondences." << std::endl;
	std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "                             each radius given by that value." << std::endl;
	std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
	std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
	std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
	std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
	std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
	std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
	std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine(int argc, char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() != 2)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	//Program behavior
	if (pcl::console::find_switch(argc, argv, "-k"))
	{
		show_keypoints_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-c"))
	{
		show_correspondences_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-r"))
	{
		use_cloud_resolution_ = true;
	}

	std::string used_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1)
	{
		if (used_algorithm.compare("Hough") == 0)
		{
			use_hough_ = true;
		}
		else if (used_algorithm.compare("GC") == 0)
		{
			use_hough_ = false;
		}
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);
		}
	}

	//General parameters
	pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
	pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
	pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);
}

double
computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
//
//int
//main(int argc, char *argv[])
//{
//	//parseCommandLine(argc, argv);
//
//	//模型点云，场景点云
//	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
//	//关键点点云
//	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
//	//法向量点云
//	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
//	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
//	//描述子点云
//	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
//	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
//
//	//载入点云
//	if (pcl::io::loadPLYFile(model_filename_, *model) < 0)
//	{
//		std::cout << "Error loading model cloud." << std::endl;
//		showHelp(argv[0]);
//		return (-1);
//	}
//	if (pcl::io::loadPLYFile(scene_filename_, *scene) < 0)
//	{
//		std::cout << "Error loading scene cloud." << std::endl;
//		showHelp(argv[0]);
//		return (-1);
//	}
//	//pcl::PointCloud<PointType>::Ptr model_tr(new pcl::PointCloud<PointType>());
//	//pcl::transformPointCloud(*model, *model_tr, Eigen::Vector3f(0, 0, -15), Eigen::Quaternionf(1, 0, 0, 0));
//	//*scene = *scene + *model_tr;
//	//pcl::io::savePCDFileBinary("GSU_scene_clu.pcd", *scene);
//	//
//	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//	norm_est.setKSearch(10);                                                                                                                              
//	norm_est.setInputCloud(model);
//	norm_est.compute(*model_normals);
//
//	norm_est.setInputCloud(scene);
//	norm_est.compute(*scene_normals);
//
//	//
//	//  Downsample Clouds to Extract keypoints
//	//
//
//	pcl::UniformSampling<PointType> uniform_sampling;
//	uniform_sampling.setInputCloud(model);
//	uniform_sampling.setRadiusSearch(model_ss_);
//	uniform_sampling.filter(*model_keypoints);
//	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;
//
//	uniform_sampling.setInputCloud(scene);
//	uniform_sampling.setRadiusSearch(scene_ss_);
//	uniform_sampling.filter(*scene_keypoints);
//	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
//
//	//pcl::io::loadPCDFile("point cloud files/GSU_model_keypoints.pcd", *model_keypoints);
//	//pcl::io::loadPCDFile("point cloud files/GSU_scene_keypoints_1.0.pcd", *scene_keypoints);
//
//	//
//	//  Compute Descriptor for keypoints
//	//
//	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//	descr_est.setRadiusSearch(descr_rad_);
//
//	descr_est.setInputCloud(model_keypoints);
//	descr_est.setInputNormals(model_normals);
//	descr_est.setSearchSurface(model);
//	descr_est.compute(*model_descriptors);
//
//	descr_est.setInputCloud(scene_keypoints);
//	descr_est.setInputNormals(scene_normals);
//	descr_est.setSearchSurface(scene);
//	descr_est.compute(*scene_descriptors);
//
//	//
//	//  Find Model-Scene Correspondences with KdTree
//	//
//	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
//	
//	pcl::KdTreeFLANN<DescriptorType> match_search;
//	match_search.setInputCloud(model_descriptors);
//
//	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//	for (size_t i = 0; i < scene_descriptors->size(); ++i)
//	{
//		std::vector<int> neigh_indices(1);
//		std::vector<float> neigh_sqr_dists(1);
//		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
//		{
//			continue;
//		}
//		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
//		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//		{
//			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//			model_scene_corrs->push_back(corr);
//		}
//	}
//	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
//	
//	//
//	//  Actual Clustering
//	//
//	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//	std::vector<pcl::Correspondences> clustered_corrs;
//
//	//  Using Hough3D
//	if (use_hough_)
//	{
//		//
//		//  Compute (Keypoints) Reference Frames only for Hough
//		//
//		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
//		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());
//
//		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
//		rf_est.setFindHoles(true);
//		rf_est.setRadiusSearch(rf_rad_);
//
//		rf_est.setInputCloud(model_keypoints);
//		rf_est.setInputNormals(model_normals);
//		rf_est.setSearchSurface(model);
//		rf_est.compute(*model_rf);
//
//		rf_est.setInputCloud(scene_keypoints);
//		rf_est.setInputNormals(scene_normals);
//		rf_est.setSearchSurface(scene);
//		rf_est.compute(*scene_rf);
//
//		//  Clustering
//		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
//		clusterer.setHoughBinSize(cg_size_);
//		clusterer.setHoughThreshold(cg_thresh_);
//		clusterer.setUseInterpolation(true);
//		clusterer.setUseDistanceWeight(false);
//
//		clusterer.setInputCloud(model_keypoints);
//		clusterer.setInputRf(model_rf);
//		clusterer.setSceneCloud(scene_keypoints);
//		clusterer.setSceneRf(scene_rf);
//		clusterer.setModelSceneCorrespondences(model_scene_corrs);
//
//		//clusterer.cluster (clustered_corrs);
//		clusterer.recognize(rototranslations, clustered_corrs);
//	}
//	else // Using GeometricConsistency
//	{
//		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//		gc_clusterer.setGCSize(cg_size_);
//		gc_clusterer.setGCThreshold(cg_thresh_);
//
//		gc_clusterer.setInputCloud(model_keypoints);
//		gc_clusterer.setSceneCloud(scene_keypoints);
//		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
//
//		//gc_clusterer.cluster (clustered_corrs);
//		gc_clusterer.recognize(rototranslations, clustered_corrs);
//	}
//
//	//
//	//  Output results
//	//
//	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
//	for (size_t i = 0; i < rototranslations.size(); ++i)
//	{
//		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
//		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
//
//		// Print the rotation matrix and translation vector
//		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
//		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
//
//		printf("\n");
//		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
//		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
//		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
//		printf("\n");
//		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
//	}
//
//	//
//	//  Visualization
//	//
//	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
//	viewer.addPointCloud(scene, "scene_cloud");
//
//	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
//
//	
//	//show_correspondences_ = true;
//	if (show_correspondences_ || show_keypoints_)
//	{
//		//  We are translating the model so that it doesn't end in the middle of the scene representation
//		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-200, 0, 5), Eigen::Quaternionf(1, 0, 0, 0));
//		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-200, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));//test
//
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 0);
//		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
//	}
//	//show_keypoints_ = true;
//	
//	if (show_keypoints_)
//	{
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
//		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
//
//		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
//		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//	}
//	pcl::Correspondences cor = *model_scene_corrs;//test
//	cout << "cor size " << model_scene_corrs->size();
//	show_correspondences_ = true;
//
//	//if (show_correspondences_)
//	//{
//	//	for (size_t j = 0; j < cor.size(); ++j)
//	//	{
//	//		std::stringstream ss_line;
//	//		ss_line << "correspondence_line" << j << "_" << j;
//	//		PointType& model_point = off_scene_model_keypoints->at(cor[j].index_query);
//	//		PointType& scene_point = scene_keypoints->at(cor[j].index_match);
//
//	//		//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//	//		viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
//	//	}
//	//}
//	for (size_t i = 0; i < rototranslations.size(); ++i)
//	{
//		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
//		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
//
//		std::stringstream ss_cloud;
//		ss_cloud << "instance" << i;
//
//		/*pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
//		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());*/
//		
//		if (show_correspondences_)
//		{
//			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
//			{
//				std::stringstream ss_line;
//				ss_line << "correspondence_line" << i << "_" << j;
//				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
//				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
//
//				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
//			}
//		}
//	}
//	
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//
//	return (0);
//}
int
main(int argc, char *argv[])
{
	//模型点云,场景点云
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_f(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<PointType>::Ptr observe_points(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr camera_points(new pcl::PointCloud<PointType>());
	//关键点点云
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	//法向量点云
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr observe_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	//描述子点云
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

	//载入点云
	if (pcl::io::loadPLYFile(model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}
	if (pcl::io::loadPLYFile(scene_filename_, *scene) < 0 || pcl::io::loadPLYFile(scene_filename_, *scene_f) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}
	std::cout << "删除叶片前 " << scene->size() << std::endl;
	//除去叶片部分
	for (size_t i = 0; i < scene->size(); ++i)
	{
		if (scene->points[i].g>scene->points[i].r)
		{
			scene->erase(scene->begin() + i);
			i--;
		}
	}
	std::cout << "删除叶片后 " << scene->size() << std::endl;

	//计算法向量
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);

	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	//选取关键点
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "模型点云总点数" << model->size() << ";场景点云关键点数: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "场景点云总点数 " << scene->size() << "; 场景点云关键点数 " << scene_keypoints->size() << std::endl;


	//计算SHOT描述子
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);

	//寻找匹配关系
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
	// 寻找每个场景点的最近点
	const int K = 1;
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(K);//存放模型中对应点位置
		std::vector<float> neigh_sqr_dists(K);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //跳过NaN
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), K, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f  /*scene->points[j].r>scene->points[j].g*/) //  欧式距离小于0.25时建立匹配
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	//目标识别识别结果储存变量
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  使用hough投票
	//计算关键点处参考系
	pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
	pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

	pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(rf_rad_);

	rf_est.setInputCloud(model_keypoints);
	rf_est.setInputNormals(model_normals);
	rf_est.setSearchSurface(model);
	rf_est.compute(*model_rf);

	rf_est.setInputCloud(scene_keypoints);
	rf_est.setInputNormals(scene_normals);
	rf_est.setSearchSurface(scene);
	rf_est.compute(*scene_rf);
	//  识别
	pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
	clusterer.setHoughBinSize(cg_size_);
	clusterer.setHoughThreshold(cg_thresh_);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(model_keypoints);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(scene_keypoints);
	clusterer.setSceneRf(scene_rf);
	clusterer.setModelSceneCorrespondences(model_scene_corrs);

	clusterer.recognize(rototranslations, clustered_corrs);

	//输出矩阵
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}

	//可视化
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 255, 255, 255);
	viewer.addPointCloud(scene_f, "scene_cloud");//加入场景点云

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	if (show_correspondences_ || show_keypoints_)
	{
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}
	//显示关键点
	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}

	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;
		//if (rotation(0, 0) == 1.0 && rotation(1, 1) == 1.0 && rotation(2, 2) == 1.0)
		//{
		//	std::cout << "舍去" << i;
		//}
		//else
		
		//pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		//viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		
		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());

				NormalType& normal_observe_point = scene_normals->at(clustered_corrs[i][j].index_match);
				//observe_normals->push_back(normal_observe_point);
				PointType& camera_point = scene_point;   
				observe_points->push_back(scene_point);
			}
		}
	}

	uniform_sampling.setInputCloud(observe_points);
	uniform_sampling.setRadiusSearch(model_ss_*10);
	uniform_sampling.filter(*observe_points);

	pcl::PointCloud<NormalType>::Ptr observe_points_normals(new pcl::PointCloud<NormalType>());
	norm_est.setKSearch(3);
	norm_est.setInputCloud(observe_points);
	norm_est.compute(*observe_points_normals);
	for (size_t j = 0; j < observe_points->size(); ++j)
	{
		std::cout << observe_points_normals->points[j].normal_x << observe_points_normals->points[j].normal_y << observe_points_normals->points[j].normal_z << endl;
	}
	pcl::visualization::PointCloudColorHandlerCustom<PointType> observe_points_color_handler(observe_points, 255, 0, 0);
	viewer.addPointCloud(observe_points, observe_points_color_handler, "observe_pointss");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "observe_pointss");

	for (int j = 0; j < observe_points->size(); j++)
	{
		PointType& camera = observe_points->points[j] ;
		camera.x += observe_points_normals->points[j].normal_x * 200;
		camera.y += observe_points_normals->points[j].normal_y * 200;
		camera.z += observe_points_normals->points[j].normal_z * 200;
		camera_points->push_back(camera);
		std::stringstream ss_line;
		ss_line << "correspondence_line" << j << "-" << j;
		PointType& observe = observe_points->points[j];
		PointType& camerat = camera_points->points[j];
		//viewer.addLine<PointType, PointType>(observe, camera, 0, 255, 0);
		std::cout << "相机位置" << j << " " << camera.x << " " << camera.y << " " << camera.z << std::endl;
		std::cout << "相机方向" << j << " " << observe_points_normals->points[j].normal_x << " " << 
			observe_points_normals->points[j].normal_y << " " << observe_points_normals->points[j].normal_z
			 << std::endl;
		if (outfile.is_open())
		{
			double c1 = observe_points_normals->points[j].normal_y*observe_points_normals->points[j].normal_y + observe_points_normals->points[j].normal_z*observe_points_normals->points[j].normal_z;
			double c2 = -observe_points_normals->points[j].normal_x*observe_points_normals->points[j].normal_y;
			double c3 = observe_points_normals->points[j].normal_x*observe_points_normals->points[j].normal_z;
			outfile << "[" << 0 << " " << c1 << " " << -observe_points_normals->points[j].normal_x << " " << camera.x << " ;"
				<< observe_points_normals->points[j].normal_z << " " << c2 << " " << -observe_points_normals->points[j].normal_y << " " << camera.y << " ;"
				<< observe_points_normals->points[j].normal_y << " " << c3 << " " << -observe_points_normals->points[j].normal_z << " " << camera.z << " ;"
				<< "]" << std::endl;
		}
	}
	outfile.close();
	pcl::visualization::PointCloudColorHandlerCustom<PointType> camera_points_color_handler(camera_points, 0, 0, 255);
	viewer.addPointCloud(camera_points, camera_points_color_handler, "camera_points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "camera_points");


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}