#include "stdafx.h"
#include <iostream>
#include <fstream>
#include<string>

#include "config.h"
#include "Calibrate.h"
#include "PairAlign.h"
#include "WriteCvmat.h"

void writeCvmat(){
	float T[] = { 0.999999, 0.00111688, 0.00132484, 0.100613,
		- 0.0011157, 0.999999, -0.000885686, 27.8251,
		- 0.00132583, 0.000884207, 0.999999, -3.30157,
		0 ,0, 0, 1 };
	float R1[] = { -0.837853, 0.190221 ,0.511682, -358.141,
		0.0692631, 0.966793, -0.245997, 111.579,
		- 0.541485 ,- 0.170669, -0.823205 ,1175.56,
		0 ,0, 0, 1 };
	CvFileStorage *fs;
	CvMat translation = cvMat(4, 4, CV_32FC1, T); // 64FC1 for double
	CvMat extrinsic1 = cvMat(4, 4, CV_32FC1, R1); // 64FC1 for double
	fs = cvOpenFileStorage("./config/extrinsic.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "translation", &translation);
	cvWrite(fs, "extrinsic1", &extrinsic1);
	cvReleaseFileStorage(&fs);
}