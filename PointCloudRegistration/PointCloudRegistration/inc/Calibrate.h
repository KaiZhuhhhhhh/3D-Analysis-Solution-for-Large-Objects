//
// Created by �쿭 on 17/4/4.
//

//#pragma once

#ifndef CALIBRATE_ZK_H
#define CALIBRATE_ZK_H
//
#include <highgui.h> 
#include "cv.h"
#include <iostream> 
#include "opencv2/opencv.hpp"

extern int Camera_ID[4] ;
extern int Cali_Pic_Num;

extern CvMat * intrinsic_matrix;                //�ڲ�������
extern CvMat * distortion_coeffs;        //����ϵ��
extern CvMat * Cam_extrinsic_matrix;  
extern CvMat * Pro_extrinsic_matrix;

extern int camera_width;//�������
extern int camera_height;
extern CvSize board_size;   //�궨��ǵ���
extern CvSize2D32f square_size;              

extern std::vector<CvMat> T_mat_4x4;					//��ת����

void inputCameraParam(CvMat * intrinsic_matrix1, CvMat * distortion_coeffs1, CvMat * Cam_extrinsic_matrix1, CvMat * Pro_extrinsic_matrix1, int cameraNumber);
void caculate_Tmat(CvMat * r_vec, CvMat * t_vec, CvMat * T_mat);
int find_rotation_mat();
void get_rotation_mat();
void get_translation_mat();

#endif