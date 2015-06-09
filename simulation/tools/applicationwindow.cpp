#include <gl/glew.h>
#include <QtWidgets>
#include "applicationwindow.h"
#include "ui_applicationwindow.h"
#include <iostream>
#include "exampleglwidget.h"



ApplicationWindow::ApplicationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ApplicationWindow)
{
    ui->setupUi(this);

	
    udpSocket = new QUdpSocket(this);

    //Bind Socket to an Address
    if(!udpSocket->bind(QHostAddress::Any, 1236))
    {
        qDebug()<<"Unable to connect to Server";
    }

    QObject::connect(udpSocket, SIGNAL(readyRead()),
                     this, SLOT(readPendingDatagrams()));



	QObject::connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(on_openAction_clicked()));

}

ApplicationWindow::~ApplicationWindow()
{
    delete ui;
}

void ApplicationWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}


void ApplicationWindow::on_startLaserButton_clicked()
{
	int Azimuthal_value = ui->AzimuthalspinBox->value();
	int Scanline_value = ui->scanLineSpinBox->value();

	ui->widget->start_laser(Azimuthal_value, Scanline_value);
	
	ui->AzimuthalFullScanLabel->setText(QString::number(Azimuthal_value));
	ui->AzimuthalFreqLabel->setText(QString::number(Azimuthal_value));

	ui->ScanLineFullScanLabel->setText(QString::number(Scanline_value));
	ui->scanLinesValueLabel->setText(QString::number(Scanline_value));

	ui->samplingFreqValueLabel->setText(QString::number(5000));
	ui->TotalPointsValueLabel->setText(QString::number(100000));
	ui->LaserXValueLabel->setText(QString::number(ui->LaserXSpinBox->value()));
	ui->LaserYValueLabel->setText(QString::number(ui->LaserYSpinBox->value()));
	ui->LaserZValueLabel->setText(QString::number(ui->LaserZSpinBox->value()));

	ui->maxRangeValueLabel->setText(QString::number(10.0));

	ui->scanModeValueLabel->setText(QString("FS"));
	ui->LaserStatusLabel->setText(QString("ON"));

}

void ApplicationWindow::on_stopLaserButton_clicked()
{
	ui->widget->stop_laser_sensor();
	ui->LaserStatusLabel->setText(QString("OFF"));
}

void ApplicationWindow::on_LaserPositionButton_clicked()
{
	float x = ui->LaserXSpinBox->value();
	float y = ui->LaserYSpinBox->value();
	float z = ui->LaserZSpinBox->value();

	ui->widget->changeSensorPosition(x, y, z);

	ui->LaserXValueLabel->setText(QString::number(x));
	ui->LaserYValueLabel->setText(QString::number(y));
	ui->LaserZValueLabel->setText(QString::number(z));
}

void ApplicationWindow::on_fullScanModeButton_clicked()
{
	//First Argument  = 0 - for FullScan Mode
	//Rest Doesn't Matter
	
	int Azimuthal_value = ui->AzimuthalspinBox->value();
	int Scanline_value = ui->scanLineSpinBox->value();

	ui->widget->changeScanMode(0, Azimuthal_value, Scanline_value, 0.0, 0.0, 0.0, 0.0);

	ui->scanModeValueLabel->setText(QString("FS"));

	ui->AzimuthalFullScanLabel->setText(QString::number(Azimuthal_value));
	ui->AzimuthalFreqLabel->setText(QString::number(Azimuthal_value));

	ui->ScanLineFullScanLabel->setText(QString::number(Scanline_value));
	ui->scanLinesValueLabel->setText(QString::number(Scanline_value));

}

void ApplicationWindow::on_boundedElevationModeButton_clicked()
{
	
	int Azimuthal_value = ui->AzimuthalspinBox->value();
	int Scanline_value = ui->scanLineSpinBox->value();

	float upper_bound = ui->UpperBoundSpinBox->value();
	float lower_bound = ui->LowerBoundSpinBox->value();
	ui->widget->changeScanMode(1, Azimuthal_value, Scanline_value, upper_bound, lower_bound, 0.0, 0.0);
	ui->scanModeValueLabel->setText(QString("BES"));
}

void ApplicationWindow::on_regionScanModeButton_clicked()
{
	
	int Azimuthal_value = ui->AzimuthalspinBox->value();
	int Scanline_value = ui->scanLineSpinBox->value();

	float lAngular = ui->AngularLeftSpinBox->value();
	float rAngular = ui->AngularRightSpinBox->value();
	float upper_bound = ui->UpperBoundRegionSpinBox->value();
	float lower_bound = ui->LowerBoundRegionSpinBox->value();

	if(lAngular>rAngular)
	{
		float temp = rAngular;
		rAngular = lAngular;
		lAngular = temp;
	}

	if((rAngular-lower_bound)>180)
	{
		float temp = rAngular;
		rAngular = lAngular;
		lAngular = temp - 360;
	}
	

	ui->widget->changeScanMode(2,  Azimuthal_value, Scanline_value, upper_bound, lower_bound, lAngular, rAngular);

	ui->scanModeValueLabel->setText(QString("RS"));
}

void ApplicationWindow::on_openAction_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"), "C:/", tr("Data Files(*.obj *.ply)"));

	ui->widget->changeModel(filename);
}



void ApplicationWindow::readPendingDatagrams()
{
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        QHostAddress sender = QHostAddress::LocalHost;
        quint16 senderPort = 1236;

        udpSocket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);

        QDataStream dStream(&datagram, QIODevice::ReadOnly);
		// the string is probably utf8 or latin
		//QByteArray buffer(6, Qt::Uninitialized);

		//dStream.readRawData(buffer.data(), 6); 
		QString string;

		dStream>>string;

	
		if(string=="RERNLS") //run sensor
		{
			int Azimuthal_value=10;
			int Scanline_value=10;

			dStream>>Azimuthal_value;
			dStream>>Scanline_value;

			ui->widget->start_laser(Azimuthal_value, Scanline_value);
	
			//Set Value to SPin box
			ui->AzimuthalspinBox->setValue(Azimuthal_value);
			ui->scanLineSpinBox->setValue(Scanline_value);

			//Set Value to Text Labels
			ui->AzimuthalFullScanLabel->setText(QString::number(Azimuthal_value));
			ui->AzimuthalFreqLabel->setText(QString::number(Azimuthal_value));

			ui->ScanLineFullScanLabel->setText(QString::number(Scanline_value));
			ui->scanLinesValueLabel->setText(QString::number(Scanline_value));

			ui->samplingFreqValueLabel->setText(QString::number(5000));
			ui->TotalPointsValueLabel->setText(QString::number(100000));
			ui->LaserXValueLabel->setText(QString::number(ui->LaserXSpinBox->value()));
			ui->LaserYValueLabel->setText(QString::number(ui->LaserYSpinBox->value()));
			ui->LaserZValueLabel->setText(QString::number(ui->LaserZSpinBox->value()));

			ui->maxRangeValueLabel->setText(QString::number(10.0));

			ui->scanModeValueLabel->setText(QString("FS"));
			ui->LaserStatusLabel->setText(QString("ON"));
		}
		else if(string=="RESTLS")//stop sensor
		{
			ui->widget->stop_laser_sensor();
			ui->LaserStatusLabel->setText(QString("OFF"));
		}
		else if(string=="RESFFS")//Full field scan
		{
			int Azimuthal_value=10;
			int Scanline_value=10;

			dStream>>Azimuthal_value;
			dStream>>Scanline_value;

			ui->widget->changeScanMode(0, Azimuthal_value, Scanline_value, 0.0, 0.0, 0.0, 0.0);

			
			//Set Value to SPin box
			ui->AzimuthalspinBox->setValue(Azimuthal_value);
			ui->scanLineSpinBox->setValue(Scanline_value);

			//Set Value to Text Labels
			ui->scanModeValueLabel->setText(QString("FS"));

			ui->AzimuthalFullScanLabel->setText(QString::number(Azimuthal_value));
			ui->AzimuthalFreqLabel->setText(QString::number(Azimuthal_value));

			ui->ScanLineFullScanLabel->setText(QString::number(Scanline_value));
			ui->scanLinesValueLabel->setText(QString::number(Scanline_value));
		}
		else if(string=="RESBES")//Bounded Elevation Scan
		{
			int Azimuthal_value = ui->AzimuthalspinBox->value();
			int Scanline_value = ui->scanLineSpinBox->value();

			float upper_bound = 45.0;
			float lower_bound = -45.0;

			dStream>>upper_bound;
			dStream>>lower_bound;


			ui->widget->changeScanMode(1, Azimuthal_value, Scanline_value, upper_bound, lower_bound, 0.0, 0.0);
			ui->scanModeValueLabel->setText(QString("BES"));

			

			//Set Value to SPin box
			ui->UpperBoundRegionSpinBox->setValue(upper_bound);
			ui->LowerBoundRegionSpinBox->setValue(lower_bound);
		}
		else if(string=="RESRES")//Region Scan
		{
			int Azimuthal_value = ui->AzimuthalspinBox->value();
			int Scanline_value = ui->scanLineSpinBox->value();

			float upper_bound = 45.0;
			float lower_bound = -45.0;

			dStream>>upper_bound;
			dStream>>lower_bound;

			float lAngular = 0.0;
			float rAngular = 90.0;

			dStream>>lAngular;
			dStream>>rAngular;

			//Set Value to SPin box
			ui->UpperBoundRegionSpinBox->setValue(upper_bound);
			ui->LowerBoundRegionSpinBox->setValue(lower_bound);
			ui->AngularLeftSpinBox->setValue(lAngular);
			ui->AngularRightSpinBox->setValue(rAngular);


			if(lAngular>rAngular)
			{
				float temp = rAngular;
				rAngular = lAngular;
				lAngular = temp;
			}

			if((rAngular-lower_bound)>180)
			{
				float temp = rAngular;
				rAngular = lAngular;
				lAngular = temp - 360;
			}
	

			ui->widget->changeScanMode(2,  Azimuthal_value, Scanline_value, upper_bound, lower_bound, lAngular, rAngular);

			ui->scanModeValueLabel->setText(QString("RS"));

		}


    }

}
