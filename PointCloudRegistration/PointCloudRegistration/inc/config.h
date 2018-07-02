
#ifndef CONFIG_H
#define CONFIG_H
#include "Calibrate.h"
#include "PairAlign.h"

extern CvMat extrinsic[12];
extern CvMat translation;//设备上升方向

//extrinsic1 表示从1号设备到2号设备的空间变换矩阵1T2
extern int horizontalScanNum;//水平采集点云数（设备数）
extern int verticalScanNum;//竖直采集点云数（上升次数
extern float risingDistance;//每次移动的距离
extern std::string horizontalFileName[6][12];
extern std::string verticalFileName[6];
void Get_ConfigArgv();
void Get_extrinsic();

#endif