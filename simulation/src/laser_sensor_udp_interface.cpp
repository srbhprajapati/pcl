#include <iostream>
#include <fstream>
#include <cstring>
#include <QDebug>
#include <pcl/simulation/laser_sensor_udp_interface.h>

#define SENDING_PORT 1235
#define SENDING_IP	QHostAddress::LocalHost

using namespace std;

pcl::simulation::LaserSensorUdpInterface::LaserSensorUdpInterface()
{
		
    socket = new QUdpSocket(this);

}

pcl::simulation::LaserSensorUdpInterface::~LaserSensorUdpInterface()
{
	socket->~QUdpSocket();
}


void pcl::simulation::LaserSensorUdpInterface::initialize()
{
	//Bind Socket to an Address
    if(!socket->bind(QHostAddress::Any, 1236))
    {
        qDebug()<<"Unable to connect to Server";
    }

    QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));

}

void pcl::simulation::LaserSensorUdpInterface::readPendingDatagrams()
{
	
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender = QHostAddress::LocalHost;
        quint16 senderPort = 1236;

        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);


		//Read Packet Header
		std::string packetHeader(datagram.constData(), 6);


		cout<<"String : "<<packetHeader<<endl;


		QString string;
	
		if(packetHeader.compare("RERNLS")==0) //run sensor
		{
			unsigned char* datagramPointer = reinterpret_cast<unsigned char*>(datagram.data());
			unsigned int sampling_frequency = datagramPointer[7]<<8 | datagramPointer[6];
			unsigned int broadcast_port	= datagramPointer[12]<<8 | datagramPointer[11];

			emit startLaser(sampling_frequency);
			//ui->widget->start_laser(Azimuthal_value, Scanline_value);

			//send Acknowledgement
			char* ackPacket = new char[6];
			memcpy(ackPacket, "EARNLS", 6);
			sendAcknowledgement(ackPacket);


		}
		else if(packetHeader.compare("RESTLS")==0)//stop sensor
		{
			emit stopLaser();

			//send Acknowledgement
			char* ackPacket = new char[6];
			memcpy(ackPacket, "EASTLS", 6);
			sendAcknowledgement(ackPacket);
		}
		else if(packetHeader.compare("RESFFS")==0)//Full field scan
		{

			unsigned char* datagramPointer = reinterpret_cast<unsigned char*>(datagram.data());
			unsigned int azimuthalValue = datagramPointer[7]<<8 | datagramPointer[6];
			unsigned int scanlineValue	= datagramPointer[9]<<8 | datagramPointer[8];


			if(azimuthalValue>0 && azimuthalValue<16 && scanlineValue>0)
			{
				emit startFullFieldScan((int)azimuthalValue, (int)scanlineValue);

				//send Acknowledgement
				char* ackPacket = new char[6];
				memcpy(ackPacket, "EASFFS", 6);
				sendAcknowledgement(ackPacket);
			}
			else
			{
				std::cout<<"Full Field Scan : Error in Azimuthal Freq / Scanline Number"<<std::endl;
			}

		}
		else if(packetHeader.compare("RESBES")==0)//Bounded Elevation Scan
		{
			
			unsigned char* datagramPointer = reinterpret_cast<unsigned char*>(datagram.data());
			unsigned int azimuthalValue = datagramPointer[7]<<8 | datagramPointer[6];
			short minElevation = datagramPointer[9]<<8 | datagramPointer[8];
			short maxElevation = datagramPointer[11]<<8 | datagramPointer[10];
			unsigned int scanlineValue	= datagramPointer[13]<<8 | datagramPointer[12];

													
			float upper_bound = ((float)maxElevation/100.0);
			float lower_bound = ((float)minElevation/100.0);

			if(azimuthalValue>0 && azimuthalValue<16 && scanlineValue>0 && upper_bound<45.0 && lower_bound>-45.0)
			{
				emit startBoundedElevationScan((int)azimuthalValue, (int)scanlineValue, upper_bound, lower_bound);

				//send Acknowledgement
				char* ackPacket = new char[6];
				memcpy(ackPacket, "EASBES", 6);
				sendAcknowledgement(ackPacket);
			}
			else
			{
				std::cout<<"Bounded Elevation Scan : Error in Azimuthal Freq / Scanline Number / Upper Bound / Lower Bound"<<std::endl;
			}

		}
		else if(packetHeader.compare("RESRES")==0)//Region Scan
		{																								   
			unsigned char* datagramPointer = reinterpret_cast<unsigned char*>(datagram.data());

			unsigned int minAzimuth = datagramPointer[7]<<8 | datagramPointer[6];
			unsigned int maxAzimuth = datagramPointer[9]<<8 | datagramPointer[8];
			short minElevation = datagramPointer[11]<<8 | datagramPointer[10];
			short maxElevation = datagramPointer[13]<<8 | datagramPointer[12];
			unsigned int azimuthalValue = datagramPointer[15]<<8 | datagramPointer[14];
			unsigned int scanlineValue	= datagramPointer[17]<<8 | datagramPointer[16];

			
			float upper_bound = ((float)maxElevation/100.0);
			float lower_bound = ((float)minElevation/100.0);

			float lAngular = ((float)maxAzimuth/100.0);
			float rAngular = ((float)minAzimuth/100.0);

			if(lAngular<rAngular)
			{
				float temp = rAngular;
				rAngular = lAngular;
				lAngular = temp;
			}

			if((lAngular-rAngular)>180)
			{
				float temp = rAngular;
				rAngular = lAngular;
				lAngular = temp;
			}
	
			
			if(azimuthalValue>0 && azimuthalValue<16 && scanlineValue>0 && upper_bound<45.0 && lower_bound>-45.0 && lAngular>0 && rAngular>0)
			{
				emit startRegionScan(azimuthalValue, scanlineValue, upper_bound, lower_bound, lAngular, rAngular);

				//send Acknowledgement
				char* ackPacket = new char[6];
				memcpy(ackPacket, "EASRES", 6);
				sendAcknowledgement(ackPacket);
			}
			else
			{
				std::cout<<"Region Scan : Error"<<std::endl; 
				std::cout<<"Azimuthal Freq : "<<azimuthalValue<<std::endl;
				std::cout<<"Scanline Number : "<<scanlineValue<<std::endl;
				std::cout<<"Upper Bound : "<<upper_bound<<std::endl;
				std::cout<<"Lower Bound : "<<lower_bound<<std::endl;
				std::cout<<"Angular Left : "<<lAngular<<std::endl;
				std::cout<<"Angular Right : "<<rAngular<<std::endl;
				
			}


		}
		else if(packetHeader.compare("RECHMD")==0)
		{
			char* datagramPointer = datagram.data();
			char* pathPointer = new char[datagram.size() - 6];

			memcpy(pathPointer, &datagramPointer[6], datagram.size() - 6);

			const char* constPathPointer = reinterpret_cast<const char*>(pathPointer);

			std::string path(constPathPointer, datagram.size()-6);

			emit changeModel(QString::fromStdString(path));

		}
		else if(packetHeader.compare("RESVMD")==0)
		{
			char* datagramPointer = datagram.data();
			char* pathPointer = new char[datagram.size() - 6];

			memcpy(pathPointer, &datagramPointer[6], datagram.size() - 6);

			const char* constPathPointer = reinterpret_cast<const char*>(pathPointer);

			std::string path(constPathPointer, datagram.size()-6);

			emit saveModel(QString::fromStdString(path));
		}

    }

}

void pcl::simulation::LaserSensorUdpInterface::sendData(char* data, int length)
{
	
	const char* finalData = data;

	int bytesWritten = socket->writeDatagram(finalData, length, SENDING_IP, SENDING_PORT);

    if(bytesWritten==-1) 
	{
		qDebug()<<"Error Sending Point Data to the Client";
	}
}

void pcl::simulation::LaserSensorUdpInterface::sendAcknowledgement(char* ack)
{
	
	const char* finalData = ack;

	int bytesWritten = socket->writeDatagram(finalData, 6, SENDING_IP, SENDING_PORT);

    if(bytesWritten==-1) 
	{
		qDebug()<<"Error Sending Point Data to the Client";
	}
}

bool pcl::simulation::LaserSensorUdpInterface::broadcastSensorData()
{

	return false;
}