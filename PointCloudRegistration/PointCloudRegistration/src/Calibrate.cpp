
#include "stdafx.h"
#include "Calibrate.h"
#include "Config.h"
#include <highgui.h> 
#include "cv.h"
#include <iostream> 
#include "opencv2/opencv.hpp"

#include <Windows.h>
#include <stdlib.h>
using namespace std;

CvSize board_size = cvSize(7, 10);   //�궨��ǵ���
CvSize2D32f square_size = cvSize2D32f(10, 10);                //���񳤿�
int camera_width = 960;//�������
int camera_height = 720;

int Camera_ID[4] ;
int Cali_Pic_Num;

CvMat * intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);                //�ڲ�������
CvMat * distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);        //����ϵ��
CvMat * Cam_extrinsic_matrix = cvCreateMat(4, 4, CV_32FC1);    //�������ϵ��ͶӰ��
CvMat * Pro_extrinsic_matrix = cvCreateMat(4, 4, CV_32FC1);    //��������ϵ��ͶӰ��

vector<CvMat> T_mat_4x4;					//��ת����


void inputCameraParam(CvMat * intrinsic_matrix1, CvMat * distortion_coeffs1, CvMat * Cam_extrinsic_matrix1, CvMat * Pro_extrinsic_matrix1, int cameraNumber)
{
	CvMat * rotation_vec = cvCreateMat(3, 1, CV_32FC1);                //��ת����
	CvMat * translation_vec = cvCreateMat(3, 1, CV_32FC1);        //ƽ�ƾ���
	CvMat *temp = cvCreateMat(2, 3, CV_32FC1);        
	string pathName;
	string fileName;

	pathName = ".//Calibration//proj" + to_string(cameraNumber) + "//";
	//��ȡ�������
	CvFileStorage *fs;

	fileName = pathName + "cam_intrinsic.xml";
	fs = cvOpenFileStorage(fileName.c_str(), 0, CV_STORAGE_READ);
	if (fs)
	{
		*intrinsic_matrix1 = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "cam_intrinsic"));
	}
	else
	{
		cout << "Error: can not find the cam_intrinsic!!!!!" << endl;
	}
	fileName = pathName + "cam_distortion.xml";
	fs = cvOpenFileStorage(fileName.c_str(), 0, CV_STORAGE_READ);
	if (fs){
		*distortion_coeffs1 = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "cam_distortion"));//����������������������ͷ�
	}
	else
	{
		cout << "Error: can not find the cam_distortion!!!!!" << endl;
	}
	fileName = pathName + "cam_extrinsic.xml";
	fs = cvOpenFileStorage(fileName.c_str(), 0, CV_STORAGE_READ);
	if (fs){
		*temp = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "cam_extrinsic"));
	}
	else
	{
		cout << "Error: can not find the cam_extrinsic!!!!!" << endl;
	}
	cvReleaseFileStorage(&fs);
	


	for (int i = 0; i < 3; i++)
	{
		CV_MAT_ELEM(*rotation_vec, float, i, 0) = CV_MAT_ELEM(*temp, float, 0, i);
		CV_MAT_ELEM(*translation_vec, float, i, 0) = CV_MAT_ELEM(*temp, float, 1, i);
	}

	caculate_Tmat(rotation_vec, translation_vec, Cam_extrinsic_matrix1);

	//��ȡͶӰ�ǲ���
	CvFileStorage *fs1;
	fileName = pathName + "proj_extrinsic.xml";
	fs1 = cvOpenFileStorage(fileName.c_str(), 0, CV_STORAGE_READ);
	if (fs1)
	{
		*temp = *cvCloneMat((CvMat *)cvReadByName(fs1, NULL, "proj_extrinsic"));
		cvReleaseFileStorage(&fs1);
	}
	else
	{
		cout << "Error: can not find the pro_extrinsic!!!!!" << endl;
	}

	for (int i = 0; i < 3; i++)
	{
		CV_MAT_ELEM(*rotation_vec, float, i, 0) = CV_MAT_ELEM(*temp, float, 0, i);
		CV_MAT_ELEM(*translation_vec, float, i, 0) = CV_MAT_ELEM(*temp, float, 1, i);
	}
	caculate_Tmat(rotation_vec, translation_vec, Pro_extrinsic_matrix1);

	cv::Mat a;
	a = intrinsic_matrix1;
	std::cout << "intrinsic:" << a << endl;
	a = distortion_coeffs1;
	std::cout << "distortion:" << a << endl;
	a = Cam_extrinsic_matrix1;
	std::cout <<"Cam_extrinsic:"<< a << endl;
	a = Pro_extrinsic_matrix1;
	std::cout << "Pro_extrinsic_matrix:" << a << endl;

	cvReleaseMat(&rotation_vec);
	cvReleaseMat(&translation_vec);
	cvReleaseMat(&temp); 
}

void caculate_Tmat(CvMat * r_vec, CvMat * t_vec, CvMat * T_mat)
{
	CvMat * r_mat = cvCreateMat(3, 3, CV_32FC1);
	cvRodrigues2(r_vec, r_mat);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			CV_MAT_ELEM(*T_mat, float, i, j) = CV_MAT_ELEM(*r_mat, float, i, j);
			CV_MAT_ELEM(*T_mat, float, 3, j) = 0;
		}
		CV_MAT_ELEM(*T_mat, float, i, 3) = CV_MAT_ELEM(*t_vec, float, i, 0);
	}
	CV_MAT_ELEM(*T_mat, float, 3, 3) = 1;
	cvReleaseMat(&r_mat);
}

int find_rotation_mat(char  cali_flag)
{	
	char t[10];	

	int imageName[2] ;
	int number_image = 2;
	char *str1;
	str1 = ".jpg";
	char filename[40] = "";

	float square_length = square_size.width;                //���񳤶�
	float square_height = square_size.height;                //����߶�
	int board_width = board_size.width;   //ÿ�нǵ���
	int board_height = board_size.height;  //ÿ�нǵ���
	int total_per_image = board_width*board_height;  //ÿ��ͼƬ�ǵ�����

	std::cout << "Index of scanners" << endl;
	cin >> t;

	imageName[0] = (t[0] - '0') * 100 + (t[1] - '0') * 10 + (t[0] - '0');
	imageName[1] = (t[0] - '0') * 100 + (t[1] - '0') * 10 + (t[1] - '0');

	if (cali_flag == '1')
	{		
		int camera_id;
		CvCapture* capture1;	
		camera_id = Camera_ID[t[0] - '0'-1];
		capture1 = cvCreateCameraCapture(camera_id);//��ȡ���id
		cvSetCaptureProperty(capture1, CV_CAP_PROP_FRAME_WIDTH, camera_width);
		cvSetCaptureProperty(capture1, CV_CAP_PROP_FRAME_HEIGHT, camera_height);
		CvCapture* capture2;
		camera_id = Camera_ID[t[1] - '0'-1];
		capture2 = cvCreateCameraCapture(camera_id);//��ȡ���id
		cvSetCaptureProperty(capture2, CV_CAP_PROP_FRAME_WIDTH, camera_width);
		cvSetCaptureProperty(capture2, CV_CAP_PROP_FRAME_HEIGHT, camera_height);

		if (capture1 == 0 || capture2 == 0)
		{
			if (capture1 == 0){
				std::cout << "�޷���������ͷ�豸��" << t[0]<<endl;
			}
			if (capture2 == 0){
				std::cout << "�޷���������ͷ�豸��" << t[1] << endl;
			}
			return 0;
		}
		else
		{
			printf("��������ͷ�豸�ɹ�����\n\n");
		}

		IplImage* frame1 = NULL;
		IplImage* frame2 = NULL;
		cvNamedWindow("�������֡��ȡ����", 1);
		cvNamedWindow("�������֡��ȡ����", 1);

		printf("����C������ȡ��ǰ֡������Ϊ�궨ͼƬ...\n");

		while (1){
			frame1 = cvQueryFrame(capture1);
			frame2 = cvQueryFrame(capture2);
			cvShowImage("�������֡��ȡ����", frame1);
			cvShowImage("�������֡��ȡ����", frame2);

			if (cvWaitKey(10) == 'c')
			{
				sprintf_s(filename, ".//Calibration//%d.jpg", imageName[0]);
				cvSaveImage(filename, frame1);
				std::cout << "�ɹ���ȡ��ǰ֡�������ļ���" << filename << "����...\n\n";

				sprintf_s(filename, ".//Calibration//%d.jpg", imageName[1]);
				cvSaveImage(filename, frame2);
				std::cout << "�ɹ���ȡ��ǰ֡�������ļ���" << filename << "����...\n\n";
				break;
			}
		}
		cvDestroyWindow("�������֡��ȡ����");
		cvDestroyWindow("�������֡��ȡ����");
		//	cvReleaseImage(&frame);
		cvReleaseCapture(&capture1);
		cvReleaseCapture(&capture2);
	}
	else if (cali_flag == '2')
	{
		number_image = 2;// Cali_Pic_Num;
	}

	IplImage * show;
	cvNamedWindow("RePlay", 1);

	int number_image_copy = number_image;  //����ͼ��֡��

	int count;  //�洢ÿ֡ͼ����ʵ��ʶ��Ľǵ���
	int found;        //ʶ��궨��ǵ�ı�־λ
	int step;        //�洢������step=successes*total_per_image;
	int successes = 0;        //�洢�ɹ��ҵ��궨�������нǵ��ͼ��֡��
	int a = 1;        //��ʱ��������ʾ�ڲ�����a֡ͼ��


	CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];   //�洢�ǵ�ͼ�����������
	CvMat * image_points = cvCreateMat(number_image*total_per_image, 2, CV_32FC1);        //�洢�ǵ��ͼ������ľ���                
	CvMat * object_points = cvCreateMat(number_image*total_per_image, 3, CV_32FC1);        //�洢�ǵ����ά����ľ���
	CvMat * point_counts = cvCreateMat(number_image, 1, CV_32SC1);                //�洢ÿ֡ͼ���ʶ��Ľǵ���

	while (a <= 2)
	{
		sprintf_s(filename, ".//Calibration//%d.jpg", imageName[a-1]);
		show = cvLoadImage(filename, -1);

		found = cvFindChessboardCorners(show, board_size, image_points_buf, &count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found == 0)
		{
			cout << "ͼ��" << imageName[a] << ".jpgͼƬ�޷��ҵ����̸����нǵ�!\n\n";
			cvNamedWindow("RePlay", 1);
			cvShowImage("RePlay", show);
			cvWaitKey(0);

		}
		else
		{
			cout << "ͼ��" << imageName[a] << ".jpg�ɹ����" << count << "���ǵ�...\n";

			cvNamedWindow("RePlay", 1);

			IplImage * gray_image = cvCreateImage(cvGetSize(show), 8, 1);
			cvCvtColor(show, gray_image, CV_BGR2GRAY);
			cout << "��ȡԴͼ��Ҷ�ͼ�������...\n";
			cvFindCornerSubPix(gray_image, image_points_buf, count, cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cout << "�Ҷ�ͼ�����ػ��������...\n";
			cvDrawChessboardCorners(show, board_size, image_points_buf, count, found);
			cout << "��Դͼ���ϻ��ƽǵ�������...\n\n";
			cvShowImage("RePlay", show);

			cvWaitKey(0);
		}

		if (total_per_image == count)//�ҵ������нǵ�
		{
			step = successes*total_per_image;
			for (int i = step, j = 0; j < total_per_image; ++i, ++j)
			{
				CV_MAT_ELEM(*image_points, float, i, 0) = image_points_buf[j].x;
				CV_MAT_ELEM(*image_points, float, i, 1) = image_points_buf[j].y;
				CV_MAT_ELEM(*object_points, float, i, 0) = (float)((j / board_width)*square_length);//��
				CV_MAT_ELEM(*object_points, float, i, 1) = (float)((j%board_width)*square_height);//��
				CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int, successes, 0) = total_per_image;
			successes++;
		}
		a++;
	}
	CvSize Pic_size = cvGetSize(show);

	cvReleaseImage(&show);
	cvDestroyWindow("RePlay");


	cout << "*********************************************\n";
	cout << number_image << "֡ͼƬ�У��궨�ɹ���ͼƬΪ" << successes << "֡...\n";
	cout << number_image << "֡ͼƬ�У��궨ʧ�ܵ�ͼƬΪ" << number_image - successes << "֡...\n\n";
	cout << "*********************************************\n\n";

	CvMat * object_points2 = cvCreateMat(total_per_image, 3, CV_32FC1);
	CvMat * image_points2 = cvCreateMat(total_per_image, 2, CV_32FC1);
	CvMat * point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	CvMat * rotation_vec = cvCreateMat(3, 1, CV_32FC1);                //��ת����
	CvMat * translation_vec = cvCreateMat(3, 1, CV_32FC1);        //ƽ�ƾ���
	CvMat * T_Mat = cvCreateMat(4, 4, CV_32FC1);        //ƽ�ƾ���

	for (int j = 0; j < successes; ++j)
	{
		for (int i = 0; i < total_per_image; ++i)
		{
			CV_MAT_ELEM(*image_points2, float, i, 0) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*image_points2, float, i, 1) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 2);
		}
		CV_MAT_ELEM(*point_counts2, int, j, 0) = CV_MAT_ELEM(*point_counts, int, j, 0);

		inputCameraParam(intrinsic_matrix, distortion_coeffs, Cam_extrinsic_matrix, Pro_extrinsic_matrix, (t[j] - '0'));
		cvFindExtrinsicCameraParams2(object_points2, image_points2, intrinsic_matrix, distortion_coeffs, rotation_vec, translation_vec);
		caculate_Tmat(rotation_vec, translation_vec, T_Mat);//������α任
		T_mat_4x4.push_back(*cvCloneMat(T_Mat));

		CvFileStorage *fs2;
		string matName = t;

		matName = "extrinsic" + matName + to_string(t[j]-'0');
		if (t[0] == '1'&&j==0){
			fs2 = cvOpenFileStorage("./Calibration/scanner_extrinsic.xml", 0, CV_STORAGE_WRITE);//���ԭ�е�
		}
		else{
			fs2 = cvOpenFileStorage("./Calibration/scanner_extrinsic.xml", 0, CV_STORAGE_APPEND);//׷��
		}
		cvWrite(fs2, matName.c_str(), T_Mat);
		cvReleaseFileStorage(&fs2);
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);

	cvReleaseMat(&rotation_vec);
	cvReleaseMat(&translation_vec);
	cvReleaseMat(&T_Mat);
}

int find_translation_mat(char  cali_flag)
{
	char rise_hight[10];

	int number_image = 2;
	char *str1;
	str1 = ".jpg";
	char filename[50] = "";

	float square_length = square_size.width;                //���񳤶�
	float square_height = square_size.height;                //����߶�
	int board_width = board_size.width;   //ÿ�нǵ���
	int board_height = board_size.height;  //ÿ�нǵ���
	int total_per_image = board_width*board_height;  //ÿ��ͼƬ�ǵ�����

	cout << "�����������߶ȣ�" << endl;
	cin >> rise_hight;

	if (cali_flag == '1')
	{
		CvCapture* capture;
		capture = cvCreateCameraCapture(Camera_ID[3]);
		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, camera_width);
		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, camera_height);
		if (capture == 0)
		{
			printf("�޷���������ͷ�豸��\n\n");
			return 0;
		}
		else
		{
			printf("��������ͷ�豸�ɹ�����\n\n");
		}

		IplImage* frame = NULL;

		cvNamedWindow("�����֡��ȡ����", 1);

		printf("����C������ȡ��ǰ֡������Ϊ�궨ͼƬ...\n����Q�����˳���ȡ֡����...\n\n");


		for (int i = 0; i < number_image; i++)
		{
			while (1){
				frame = cvQueryFrame(capture);
				if (!frame)
					break;
				cvShowImage("�����֡��ȡ����", frame);

				if (cvWaitKey(10) == 'c')
				{
					sprintf_s(filename, ".//Calibration//translation%d.jpg", i);
					cvSaveImage(filename, frame);
					cout << "�ɹ���ȡ��ǰ֡�������ļ���" << filename << "����...\n\n";
					printf("����C������ȡ��ǰ֡������Ϊ�궨ͼƬ...\n");
					
					cout << "��̧��ɨ���ǣ�Ȼ�������գ�" << endl;
					break;
				}
				//else if (cvWaitKey(10) == 'q')
				//{
				//	printf("��ȡͼ��֡�������...\n\n");
				//	cout << "���ɹ���ȡ" << --number_image << "֡ͼ�񣡣�\n\n";
				//	break;
				//}
			}
		}

		cvDestroyWindow("�����֡��ȡ����");
		//	cvReleaseImage(&frame);
		cvReleaseCapture(&capture);
	}

	IplImage * show;
	cvNamedWindow("RePlay", 1);

	int number_image_copy = number_image;  //����ͼ��֡��

	int count;  //�洢ÿ֡ͼ����ʵ��ʶ��Ľǵ���
	int found;        //ʶ��궨��ǵ�ı�־λ
	int step;        //�洢������step=successes*total_per_image;
	int successes = 0;        //�洢�ɹ��ҵ��궨�������нǵ��ͼ��֡��
	int a = 0;        //��ʱ��������ʾ�ڲ�����a֡ͼ��


	CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];   //�洢�ǵ�ͼ�����������
	CvMat * image_points = cvCreateMat(number_image*total_per_image, 2, CV_32FC1);        //�洢�ǵ��ͼ������ľ���                
	CvMat * object_points = cvCreateMat(number_image*total_per_image, 3, CV_32FC1);        //�洢�ǵ����ά����ľ���
	CvMat * point_counts = cvCreateMat(number_image, 1, CV_32SC1);                //�洢ÿ֡ͼ���ʶ��Ľǵ���

	while (a < number_image_copy)
	{
		sprintf_s(filename, ".//Calibration//translation%d.jpg", a);
		show = cvLoadImage(filename, -1);

		found = cvFindChessboardCorners(show, board_size, image_points_buf, &count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found == 0)
		{
			cout << "��" << a << "֡ͼƬ�޷��ҵ����̸����нǵ�!\n\n";
			cvNamedWindow("RePlay", 1);
			cvShowImage("RePlay", show);
			cvWaitKey(0);

		}
		else
		{
			cout << "��" << a << "֡ͼ��ɹ����" << count << "���ǵ�...\n";

			cvNamedWindow("RePlay", 1);

			IplImage * gray_image = cvCreateImage(cvGetSize(show), 8, 1);
			cvCvtColor(show, gray_image, CV_BGR2GRAY);
			cout << "��ȡԴͼ��Ҷ�ͼ�������...\n";
			cvFindCornerSubPix(gray_image, image_points_buf, count, cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cout << "�Ҷ�ͼ�����ػ��������...\n";
			cvDrawChessboardCorners(show, board_size, image_points_buf, count, found);
			cout << "��Դͼ���ϻ��ƽǵ�������...\n\n";
			cvShowImage("RePlay", show);

			cvWaitKey(0);
		}

		if (total_per_image == count)//�ҵ������нǵ�
		{
			step = successes*total_per_image;
			for (int i = step, j = 0; j < total_per_image; ++i, ++j)
			{
				CV_MAT_ELEM(*image_points, float, i, 0) = image_points_buf[j].x;
				CV_MAT_ELEM(*image_points, float, i, 1) = image_points_buf[j].y;
				CV_MAT_ELEM(*object_points, float, i, 0) = (float)((j / board_width)*square_length);//��
				CV_MAT_ELEM(*object_points, float, i, 1) = (float)((j%board_width)*square_height);//��
				CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int, successes, 0) = total_per_image;
			successes++;
		}
		a++;
	}
	CvSize Pic_size = cvGetSize(show);

	cvReleaseImage(&show);
	cvDestroyWindow("RePlay");


	cout << "*********************************************\n";
	cout << number_image << "֡ͼƬ�У��궨�ɹ���ͼƬΪ" << successes << "֡...\n";
	cout << number_image << "֡ͼƬ�У��궨ʧ�ܵ�ͼƬΪ" << number_image - successes << "֡...\n\n";
	cout << "*********************************************\n\n";

	CvMat * object_points2 = cvCreateMat(total_per_image, 3, CV_32FC1);
	CvMat * image_points2 = cvCreateMat(total_per_image, 2, CV_32FC1);
	CvMat * point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	CvMat * rotation_vec = cvCreateMat(3, 1, CV_32FC1);                //��ת����
	CvMat * translation_vec = cvCreateMat(3, 1, CV_32FC1);        //ƽ�ƾ���
	CvMat * T_Mat[2];
	T_Mat[0] = cvCreateMat(4, 4, CV_32FC1);        //ƽ�ƾ���
	T_Mat[1] = cvCreateMat(4, 4, CV_32FC1);        //ƽ�ƾ���

	for (int j = 0; j < successes; ++j)
	{
		for (int i = 0; i < total_per_image; ++i)
		{
			CV_MAT_ELEM(*image_points2, float, i, 0) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*image_points2, float, i, 1) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 2);
		}
		CV_MAT_ELEM(*point_counts2, int, j, 0) = CV_MAT_ELEM(*point_counts, int, j, 0);
		inputCameraParam(intrinsic_matrix, distortion_coeffs, Cam_extrinsic_matrix, Pro_extrinsic_matrix, 4);
		cvFindExtrinsicCameraParams2(object_points2, image_points2, intrinsic_matrix, distortion_coeffs, rotation_vec, translation_vec);
		caculate_Tmat(rotation_vec, translation_vec, T_Mat[j]);//������α任
		
	}
	CvFileStorage *fs2;
	CvMat * TranslationMat=cvCreateMat(4, 4, CV_32FC1);        //ƽ�ƾ���
	CvMat* c1Tb_inv = cvCreateMat(4, 4, CV_32FC1);
	string matName;
	cv::Mat debug;//������
	cvInvert(T_Mat[0], c1Tb_inv);
	cvmMul(T_Mat[1], c1Tb_inv, TranslationMat);//�ѵײ��ƶ����ϲ�
	debug = TranslationMat;
	cout << "TranslationMat" << debug<<endl;
	cvConvertScale(TranslationMat, &translation, 1 / atof(rise_hight));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			CV_MAT_ELEM(translation, float, i, j) = 0;
		}
	}
	CV_MAT_ELEM(translation, float, 3, 3) = 0;
	debug = &translation;
	cout << "translation" << debug << endl;
	matName = "translation";
	
	fs2 = cvOpenFileStorage("./Calibration/translation_extrinsic.xml", 0, CV_STORAGE_WRITE);//���ԭ�е�
	cvWriteReal(fs2, "rise_hight", atof(rise_hight));
	cvWrite(fs2, matName.c_str(), &translation);

	cvReleaseFileStorage(&fs2);
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);

	cvReleaseMat(&rotation_vec);
	cvReleaseMat(&translation_vec);
}

void get_rotation_mat(){
	cv::Mat a;//������
	char caliModel;
	std::cout << "1 Calibration Scanner extrinsic" << endl;
	std::cout << "2 Calculate scanner extrinsic from captured picture" << endl;
	std::cout << "3 Scanner Extrinsic matrix existed" << endl;
	cin >> caliModel;

	if (caliModel != '1' && caliModel != '2' && caliModel != '3'){ cout << "wrong input!" << endl; exit(0); }

	if (caliModel == '3'){
		CvMat *temp0;// = cvCreateMat(4, 4, CV_32FC1);;
		CvFileStorage *fs0;
		string extrinsicMatName;
		fs0 = cvOpenFileStorage("./Calibration/scanner_extrinsic.xml", 0, CV_STORAGE_READ);
		for (int i = 1; i < horizontalScanNum; i++){
			extrinsicMatName = "extrinsic" + to_string(i) + to_string(i + 1) + to_string(i);//�������ϵ�������궨������ϵ�����ϱ�Ϊ���
			temp0 = (CvMat *)cvReadByName(fs0, NULL, extrinsicMatName.c_str());
			a = temp0;
			std::cout << extrinsicMatName << ":" << a << endl;
			T_mat_4x4.push_back(*cvCloneMat(temp0));
			extrinsicMatName = "extrinsic" + to_string(i) + to_string(i + 1) + to_string(i+1);
			temp0 = (CvMat *)cvReadByName(fs0, NULL, extrinsicMatName.c_str());
			a = temp0;
			std::cout << extrinsicMatName << ":" << a << endl;
			T_mat_4x4.push_back(*cvCloneMat(temp0));
		}
	}
	else{
		for (int i = 1; i < horizontalScanNum; i++){
			find_rotation_mat(caliModel);
		}
	}
	for (int i = 0; i <horizontalScanNum-1; i++){	
		CvMat* T1_inv = cvCreateMat(4, 4, CV_32FC1);
		CvMat* T1 = &T_mat_4x4[i * 2 ];		
		CvMat* T2 = &T_mat_4x4[i * 2 + 1];
		CvMat* scanner_extrinsic = cvCreateMat(4, 4, CV_32FC1);
		a = T1;
		std::cout << "T1" <<":" << a << endl;
		a = T2;
		std::cout << "T2" << ":" << a << endl;
		cvInvert(T1, T1_inv);
		cvmMul(T2, T1_inv, scanner_extrinsic);
		extrinsic[i] = *cvCloneMat(scanner_extrinsic);
		a = &(extrinsic[i]);
		std::cout << "scanner_extrinsic" << i << i+1<<":" << a << endl;
	}
}

void get_translation_mat(){
	cv::Mat a;//������
	char caliModel;
	std::cout << "1 Calibration translation" << endl;
	std::cout << "2 Calculate translation extrinsic from captured picture" << endl;
	std::cout << "3 Translation Extrinsic matrix existed" << endl;
	cin >> caliModel;

	if (caliModel != '1' && caliModel != '2' && caliModel != '3'){ cout << "wrong input!" << endl; exit(0); }

	if (caliModel == '3'){
		CvFileStorage *fs0;
		string extrinsicMatName;
		fs0 = cvOpenFileStorage("./Calibration/translation_extrinsic.xml", 0, CV_STORAGE_READ);
	
		extrinsicMatName = "translation"  ;//�������ϵ�������궨������ϵ�����ϱ�Ϊ���
		translation = *(CvMat *)cvReadByName(fs0, NULL, extrinsicMatName.c_str());
		a = &translation;
		std::cout << extrinsicMatName << ":" << a << endl;		
	}
	else{
		find_translation_mat(caliModel);
	}
}