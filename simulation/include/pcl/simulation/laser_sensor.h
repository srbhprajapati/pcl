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

#include <QtNetwork/QUdpSocket>

#define LOCALHOST_IP QHostAddress::LocalHost
#define LOCALHHOST_PORT 1235
#define PI 3.14159265

//Parameters Related to Sensor
#define FARTHEST_DISTANCE 10.0


using namespace std;


namespace pcl
{
	namespace simulation
	{
		class PCL_EXPORTS LaserSensor : public QObject
		{
			Q_OBJECT
			public:
				
				//Total number of scanlines
				int scanLines;
				
				//Frequency at which the sensor is sampling points
				int samplingFrequency;

				//Shader IDs for the OpenGL rendering - should be private
				GLuint ProgramId, VertexShaderId, FragmentShaderId;

				//Near Plane, Far Plane - don't know azimuth angle
				float znear, zfar, main_azimuthal_angle;

				//should be private
				int index;

				//frequency at which the sensor is rotating its head
				int azimuthal_frequency;

				
				//scanLineAdd - Whether the sensor is moving from top to bottom
				//				or bottom to top				
				int scanLineIndex, scanLineAdd;

				//Laser sensor Position from where we are sensing the environment
				float CameraPosition[3];
				
				LaserSensor()
				{
					//Initialization of Variables for an instance of the
					//laser sensor class. These variables will determine
					//the scanning behavior(not scan pattern but other properties
					//range, accuracy etc.)
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
				

				/* Renders the scene to Depth Texture using 'Render to Texture' feature 
				   of OpenGL. This function creates depth textures relative to the current
				   position of the sensor. If one has to increase the accuracy of the texture,
				   he would have to use a 16 bit depth buffer instead of 8 Bit. On the other hand
				   if someone has to increase the resolution of the sensor, then he would have
				   to increase the width and height of the texture on which the scene is rendering.
				   */
				void renderSceneToDepthTexture(GLuint &dfbo, 
												float lookAt[],
												float CameraPosition[],
												int texture_width, 
												int texture_height,
												Scene::Ptr scene_,
												int window_width,
												int window_height);


				
				/*
				 *	This function converts the current depth textures to its
				 *	actual depth value using camera information like near plane
				 *	and far plane.
				 */
				void depthTextureToRealDepthValues(GLuint &dtexture, 
													GLuint &dfbo_1,
													GLuint VBO,
													int window_height,
													int window_width,
													int texture_width,
													int texture_height);

				
				/*
				 *	Method for generating textures (Scene Depth Texture and Offset Texture) 
				 *	for scene rendering.
				 */
				void generatetextures(GLuint &dtexture, 
										GLuint &dtexture_1, 
										GLuint &dfbo, 
										GLuint &dfbo_1,
										int texture_width,
										int texture_height);



				/*
				 *	Creation of Vertex shader and fragment shader for rendering
				 */
				void CreateShaders();



				//Not used I think - Check again				
				/*void getPointCloud(GLuint &dtexture_1, 
									int index,
									int texture_width,
									int texture_height,
									float CameraPosition[],
									float *points);
*/

				/*
					Returns the Point Cloud for Full Field Scan Pattern
				*/
				void generateRE0xPointCloudFullScan(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points		
							);

				
				/*
					Returns the Point Cloud for Bounded Elevation Scan Pattern
				*/
				void generateRE0xPointCloudBoundedElevation(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points,
							float upperBound,
							float lowerBound
							);

				/*
					Returns the Point Cloud for Region Scan Pattern
				*/
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



				
				/*
					Display Message
				*/
				void printMessage(void) {cout<<"laser sensor printf working"<<endl;}
				
				
				/*
				Initialize the sensor to setup the sensor in the 3D space.
				Defines the vertex and Fragment shader for the sensor. Ray marching for
				the generation of sensor data is performed in Fragment Shader
				*/
				//void initSensor(void);

				/*
				Main depth Map display Loop:
				Generate Depth map and display it on the OpenGL Window.
				*/
				void generateData(void);
				

				/*
					Sending the Point Cloud Data to the Client over UDP.
				*/				
				void sendData(QByteArray data);

			signals:
				void sendData(QString packetToBeSent);		

			private:
			
			int width_;
			int height_;
			float xpos_;
			float ypos_;
			float zpos_;
			
			QUdpSocket *socket_;

			/*
			Intialize the sensor for UDP Transfer
			*/
			void initialize_();

		};
	}
}

#endif