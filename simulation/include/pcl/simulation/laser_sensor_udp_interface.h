

#ifndef _LASER_SENSOR_UDP_INTERFACE_HPP_
#define _LASER_SENSOR_UDP_INTERFACE_HPP_

#include <QtNetwork/QUdpSocket>
#include <QByteArray>


namespace pcl
{
	namespace simulation
	{
		class LaserSensorUdpInterface : public QObject
		{
			Q_OBJECT
			public:
				
				//constructor
				LaserSensorUdpInterface();
				
				//destructor
				~LaserSensorUdpInterface();
				
				//Initalize UDP socket				
				void initialize();

				//broadcasts sensor data using mutliple packets
				bool broadcastSensorData();
			

			signals:

				void startLaser(int Sampling_Frequency);

				void startFullFieldScan(int Azimuthal_value, int Scanline_value);

				void startBoundedElevationScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound);

				void startRegionScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound, float lAngular, float rAngular);

				void stopLaser();

				void changeModel(QString path);

				void saveModel(QString path);

			private slots:
				//reads the datagrams sent by Client
				void readPendingDatagrams();

				//void sendData(QByteArray data);
				void sendData(char* data, int length);

				void sendAcknowledgement(char* ack);


			private:
				//socket for writing UDP packets
				QUdpSocket *socket;
			
		};
	}
}

#endif //_LASER_SENSOR_UDP_INTERFACE_HPP_