// 定义控制台应用程序的入口点。
//关掉了Microsoft符号调试器
//
// Created by 朱凯 on 17/4/4.
//
#include "stdafx.h"

#include "PairAlign.h"
#include "config.h"

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
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //模型
	loadData2(horizontalScanNum, horizontalFileName , data); //读取pcd文件数据，定义见上面	
	HorizontalAccurateRegistration(data);

	VerticalAccurateRegistration(data);
	//AccurateRegistration2(data);//精细拼接

	return 0;
}

/*问题日志：矩阵读取错误*/