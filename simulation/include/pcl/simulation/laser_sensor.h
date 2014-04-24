

#ifndef _LASER_SENSOR_HPP_
#define _LASER_SENSOR_HPP_


#include <pcl/pcl_macros.h>
using namespace std;


namespace pcl
{
	namespace simulation
	{
		class PCL_EXPORTS LaserSensor
		{
			public:
				
				/*
				Set the window size for which the Depth map needs to be generated.
				Sets the height and width of the depth map.
				*/
				void setSensorWindow(int height, int width){width_ = width; height_ = height;}
				
				/*
				Sets the position(x,y,z) of the sensor in 3D Space.
				*/
				void setSensorPosition(float xpos, float ypos, float zpos){xpos_ = xpos; ypos_ = ypos; zpos_ = zpos;}
				
				/*
					Display Message
				*/
				void printMessage(void) {cout<<"laser sensor printf working"<<endl;}
				
				
				/*
				Initialize the sensor to setup the sensor in the 3D space.
				Defines the vertex and Fragment shader for the sensor. Ray marching for
				the generation of sensor data is performed in Fragment Shader
				*/
				void initSensor(void);

				/*
				Main depth Map display Loop:
				Generate Depth map and display it on the OpenGL Window.
				*/
				void generateData(void);
				
			private:
			
			int width_;
			int height_;
			float xpos_;
			float ypos_;
			float zpos_;
			
		};
	}
}

#endif