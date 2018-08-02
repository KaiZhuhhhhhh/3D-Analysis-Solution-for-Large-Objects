// �������̨Ӧ�ó������ڵ㡣
//�ص���Microsoft���ŵ�����
//
// Created by �쿭 on 17/4/4.
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
	std::cout << "ɨ����λ�˹�ϵ��ȡ" << endl;
	std::cout << "1 ��־�㷨" << endl;
	std::cout << "2 ���λ�˱궨��" << endl;
	cin >> getExtrinsic;
	if(getExtrinsic == '2'){
		get_translation_mat();
		get_rotation_mat();
	}
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //ģ��
	for (int j = 1; j <= verticalScanNum; j++){
		std::string outputName = std::to_string(j)+"_layer";
		loadData2(horizontalScanNum, horizontalFileName[j-1], data); //��ȡpcd�ļ����ݣ����������	
		HorizontalAccurateRegistration(data, outputName);
		data.clear();
	}
	
	loadData2(horizontalScanNum, verticalFileName, data);
	VerticalAccurateRegistration(data);
	data.clear();
	return 0;
}