

#ifndef _LASER_SENSOR_UDP_INTERFACE_HPP_
#define _LASER_SENSOR_UDP_INTERFACE_HPP_

#include <QtNetwork/QUdpSocket>


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
				
				
				//broadcasts sensor data using mutliple packets
				bool broadcastSensorData();
				
			private:
				//socket for writing UDP packets
				QUdpSocket *socket;
			
		};
	}
}

#endif //_LASER_SENSOR_UDP_INTERFACE_HPP_