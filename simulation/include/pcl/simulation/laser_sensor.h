

#ifndef _LASER_SENSOR_HPP_
#define _LASER_SENSOR_HPP_

#include <stdlib.h>
#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif
#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif
#ifdef GLUT_IS_A_FRAMEWORK
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif

#include <pcl/pcl_macros.h>
#include <pcl/simulation/scene.h>
//#include <pcl/simulation/laser_sensor_udp_interface.h>

#include <QtNetwork/QUdpSocket>

#define LOCALHOST_IP QHostAddress::LocalHost
#define LOCALHHOST_PORT 1235
#define PI 3.14159265

using namespace std;


namespace pcl
{
	namespace simulation
	{
		class PCL_EXPORTS LaserSensor : public QObject
		{
			Q_OBJECT
			public:
								
				int scanLines;
				int samplingFrequency;
				GLuint ProgramId, VertexShaderId, FragmentShaderId;
				float znear, zfar, main_azimuthal_angle;
				//GLuint depth_texture[4], depth_texture_1[4], fbo_[4] , fbo_1[4];
				int index;
				int scanLineIndex, azimuthal_frequency, scanLineAdd;

				float CameraPosition[3];
				
				LaserSensor()
				{
					index=0;
					scanLineIndex = 32;
					znear = 1.0f;
					zfar = 10.0f;
					scanLines = 64;
					scanLineAdd=1;
					samplingFrequency = 5000;
					azimuthal_frequency = 5;
					main_azimuthal_angle = 0.0;
					CameraPosition[0] = 0.0f;
					CameraPosition[1] = 2.0f;
					CameraPosition[2] = 0.0f;

					initialize_();

					//udpSocket_ = new LaserSensorUdpInterface();
				}



				

				/*
				Set the window size for which the Depth map needs to be generated.
				Sets the height and width of the depth map.
				*/
				void setSensorWindow(int height, int width){width_ = width; height_ = height;}
				
				/*
				Sets the position(x,y,z) of the sensor in 3D Space.
				*/
				void setSensorPosition(float xpos, float ypos, float zpos){xpos_ = xpos; ypos_ = ypos; zpos_ = zpos;}
				

				void renderSceneToDepthTexture(GLuint &dfbo, 
												float lookAt[],
												float CameraPosition[],
												int texture_width, 
												int texture_height,
												Scene::Ptr scene_,
												int window_width,
												int window_height);


				
				
				void depthTextureToRealDepthValues(GLuint &dtexture, 
													GLuint &dfbo_1,
													GLuint VBO,
													int window_height,
													int window_width,
													int texture_width,
													int texture_height);

				
				void generatetextures(GLuint &dtexture, 
										GLuint &dtexture_1, 
										GLuint &dfbo, 
										GLuint &dfbo_1,
										int texture_width,
										int texture_height);


				void CreateShaders();



				
				void getPointCloud(GLuint &dtexture_1, 
									int index,
									int texture_width,
									int texture_height,
									float CameraPosition[],
									float *points);

				void generateRE0xPointCloudFullScan(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points		
							);

				
				void generateRE0xPointCloudBoundedElevation(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points,
							float upperBound,
							float lowerBound
							);

				void generateRE0xPointCloudRegionScan(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points,
							float upperBound,
							float lowerBound,
							float angularRight,
							float angularLeft
							);

				void generatePointCloud();
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
				

				
				void sendData(QByteArray data);


			private:
			
			int width_;
			int height_;
			float xpos_;
			float ypos_;
			float zpos_;
			
			QUdpSocket *socket_;
			//LaserSensorUdpInterface *udpSocket_;


			/*
			Intialize the sensor for UDP Transfer
			*/
			void initialize_();

		};
	}
}

#endif