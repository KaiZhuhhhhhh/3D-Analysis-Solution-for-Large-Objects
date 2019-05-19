#include "controlmodule.h"
#include <QMessageBox>
#include <QFileDialog>
#include <qdatetime.h>

controlModule::controlModule(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui->ProcessBarButton->setRange(0, 500 - 1);
	ui->ProcessBarButton->setVisible(false);
	ui->ProcessBarButton->setValue(0);

	ui->dateTimeEdit->setDateTime(QDateTime::currentDateTime());

	process = new QProcess();

	//���ҿ��õĴ���
	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(info);
		if (serial.open(QIODevice::ReadWrite))
		{
			ui->port_number->addItem(serial.portName());
			serial.close();
		}
	}
	//���ò����������˵�Ĭ����ʾ
	QString str[] = { "300", "1200", "9600", "19200" };
	for (int i = 0; i < 4; i++)
	{
		ui->port_baudate->addItem(str[i]);
	}
	ui->port_baudate->setCurrentIndex(2);

	connect(ui->OpenPortButton, SIGNAL(clicked()), this, SLOT(on_OpenPortButton()), Qt::UniqueConnection);
	connect(ui->CloseButton, SIGNAL(clicked()), this, SLOT(on_CloseButton()), Qt::UniqueConnection);

	connect(ui->LaserOpenButton, SIGNAL(clicked()), this, SLOT(on_LaserOpenButton()), Qt::UniqueConnection);
	connect(ui->LaserCloseButton, SIGNAL(clicked()), this, SLOT(on_LaserCloseButton()), Qt::UniqueConnection);

	connect(ui->OnButtonUp1, SIGNAL(clicked()), this, SLOT(on_ClickedButtonUp1()), Qt::UniqueConnection);
	connect(ui->OnButtonDown1, SIGNAL(clicked()), this, SLOT(on_ClickedButtonDown1()), Qt::UniqueConnection);
	connect(ui->OnButtonStop1, SIGNAL(clicked()), this, SLOT(on_ClickedButtonStop1()), Qt::UniqueConnection);

	connect(ui->OnButtonUp2, SIGNAL(clicked()), this, SLOT(on_ClickedButtonUp2()), Qt::UniqueConnection);
	connect(ui->OnButtonDown2, SIGNAL(clicked()), this, SLOT(on_ClickedButtonDown2()), Qt::UniqueConnection);
	connect(ui->OnButtonStop2, SIGNAL(clicked()), this, SLOT(on_ClickedButtonStop2()), Qt::UniqueConnection);

	connect(ui->OnButtonUp3, SIGNAL(clicked()), this, SLOT(on_ClickedButtonUp3()), Qt::UniqueConnection);
	connect(ui->OnButtonDown3, SIGNAL(clicked()), this, SLOT(on_ClickedButtonDown3()), Qt::UniqueConnection);
	connect(ui->OnButtonReset3, SIGNAL(clicked()), this, SLOT(on_ClickedButtonReset3()), Qt::UniqueConnection);
	connect(ui->OnButtonStop3, SIGNAL(clicked()), this, SLOT(on_ClickedButtonStop3()), Qt::UniqueConnection);

	connect(ui->OnButtonUp4, SIGNAL(clicked()), this, SLOT(on_ClickedButtonUp4()), Qt::UniqueConnection);
	connect(ui->OnButtonDown4, SIGNAL(clicked()), this, SLOT(on_ClickedButtonDown4()), Qt::UniqueConnection);
	connect(ui->OnButtonReset4, SIGNAL(clicked()), this, SLOT(on_ClickedButtonReset4()), Qt::UniqueConnection);
	connect(ui->OnButtonStop4, SIGNAL(clicked()), this, SLOT(on_ClickedButtonStop4()), Qt::UniqueConnection);

	connect(process, SIGNAL(error(QProcess::ProcessError)), this, SLOT(processError(QProcess::ProcessError)), Qt::UniqueConnection);
}

controlModule::~controlModule()
{

}
void controlModule::on_OpenPortButton()
{
	if (ui->OpenPortButton->text() == tr("Open"))
	{
		serial = new QSerialPort;
		//���ô�������
		serial->setPortName(ui->port_number->currentText());
		//�Զ�д��ʽ����
		serial->open(QIODevice::ReadWrite);
		//���ò�����
		serial->setBaudRate(ui->port_baudate->currentText().toInt());
		//����8����λ��
		serial->setDataBits(QSerialPort::Data8);
		//������żУ��
		serial->setParity(QSerialPort::NoParity);
		//����1λֹͣλ
		serial->setStopBits(QSerialPort::OneStop);
		//����������
		serial->setFlowControl(QSerialPort::NoFlowControl);

		ui->OpenPortButton->setText(tr("Close"));
	}
	else
	{
		//�رմ���
		serial->close();
		ui->port_baudate->setEnabled(true);
		ui->port_number->setEnabled(true);
		ui->OpenPortButton->setText(tr("Open"));
	}
}

//�رմ���,�˳�����
void controlModule::on_CloseButton()
{
	close();
	//�ر��ⲿ����
	if (process)
	{
		process->close();
		delete process;
		process = NULL;
	}
}

//���ⷢ������
void controlModule::on_LaserOpenButton()
{
	QByteArray dataSend = "C0040L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}

void controlModule::on_LaserCloseButton()
{
	QByteArray dataSend = "C0080L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}

//����˿�ܲۺ���
void controlModule::on_ClickedButtonUp1()
{
	QByteArray dataSend = "C0100L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonDown1()
{
	QByteArray dataSend = "C0200L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonStop1()
{
	QByteArray dataSend = "C0400L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}

//����˿�ܲۺ���
void controlModule::on_ClickedButtonUp2()
{
	QByteArray dataSend = "C0800L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonDown2()
{
	QByteArray dataSend = "C1000L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonStop2()
{
	QByteArray dataSend = "C2000L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}

//Զ����˿�ܲۺ���
void controlModule::on_ClickedButtonUp3()
{
	QByteArray dataSend = "C0004L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonDown3()
{
	QByteArray dataSend = "C0008L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonReset3()
{
	QByteArray dataSend = "C0010L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonStop3()
{
	QByteArray dataSend = "C0020L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}

//������˿�ܲۺ���
void controlModule::on_ClickedButtonUp4()
{
	QByteArray dataSend = "C4000L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonDown4()
{
	QByteArray dataSend = "C8000L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonReset4()
{
	QByteArray dataSend = "C0001L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}
void controlModule::on_ClickedButtonStop4()
{
	QByteArray dataSend = "C0002L";
	if (ui->OpenPortButton->text() == "Close")
	{
		serial->write(dataSend);
	}
}