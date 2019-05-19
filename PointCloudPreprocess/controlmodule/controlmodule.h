#ifndef CONTROLMODULE_H
#define CONTROLMODULE_H

#include <QtWidgets/QMainWindow>
#include "ui_controlmodule.h"
#include <QtWidgets/QMainWindow>
#include <QtMultimedia/QMediaPlayer>
#include <QtMultimediaWidgets/QVideoWidget>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>
#include <qprocess.h>

#include <QCamera>  
#include <QCameraViewfinder>  
#include <QCameraInfo> 
#include<QCameraImageCapture>
#include<QImage>
#include<qdebug.h>
#include<QMessageBox>
class controlModule : public QMainWindow
{
	Q_OBJECT

public:
	controlModule(QWidget *parent = 0);
	~controlModule();

private:
	Ui::PlantScannerClass ui;
	QSerialPort *serial;
	QProcess *process;

	private slots:

	void on_OpenPortButton();
	void on_CloseButton();

	//激光发射器开关
	void on_LaserOpenButton();
	void on_LaserCloseButton();

	//顶部丝杠槽函数
	void on_ClickedButtonUp1();
	void on_ClickedButtonDown1();
	void on_ClickedButtonStop1();

	//激光丝杠槽函数
	void on_ClickedButtonUp2();
	void on_ClickedButtonDown2();
	void on_ClickedButtonStop2();

	//远柜门丝杠槽函数
	void on_ClickedButtonUp3();
	void on_ClickedButtonDown3();
	void on_ClickedButtonReset3();
	void on_ClickedButtonStop3();

	//近柜门丝杠槽函数
	void on_ClickedButtonUp4();
	void on_ClickedButtonDown4();
	void on_ClickedButtonReset4();
	void on_ClickedButtonStop4();

	////点云获取槽函数
	//void on_GetPointCloudButton();

	//void initialVtkWidget();
	//void on_PointCloudViewButton1();
	//void on_PointCloudViewButton2();
	//void on_PointCloudViewButton3();
	//void on_PointCloudViewButton4();
	//void on_PointCloudViewButton5();
	//void on_PointCloudViewButton6();
	//
	//void on_ImageButton();
	//void on_ImageCapture();
	//void displayImage(int,QImage);
	void processError(QProcess::ProcessError);
};

#endif // CONTROLMODULE_H
