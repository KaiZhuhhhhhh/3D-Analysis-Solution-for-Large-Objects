// 定义控制台应用程序的入口点。
//关掉了Microsoft符号调试器
//
// Created by 朱凯 on 17/4/4.
//
#include "stdafx.h"

#include "PairAlign.h"
#include "config.h"
#include "Calibrate.h"

#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>  

#include <cv.h>
#include <highgui.h>

int _tmain(int argc, char** argv)
{
	Get_extrinsic();
	Get_ConfigArgv();

	char getExtrinsic;
	std::cout << "扫描仪位姿关系获取" << endl;
	std::cout << "1 标志点法" << endl;
	std::cout << "2 相机位姿标定法" << endl;
	cin >> getExtrinsic;
	if(getExtrinsic == '2'){
		get_translation_mat();
		get_rotation_mat();
	}
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //模型
	for (int j = 1; j <= verticalScanNum; j++){
		std::string outputName = std::to_string(j)+"_layer";
		loadData2(horizontalScanNum, horizontalFileName[j-1], data); //读取pcd文件数据，定义见上面	
		HorizontalAccurateRegistration(data, outputName);
		data.clear();
	}
	
	loadData2(horizontalScanNum, verticalFileName, data);
	VerticalAccurateRegistration(data);
	data.clear();
	return 0;
}