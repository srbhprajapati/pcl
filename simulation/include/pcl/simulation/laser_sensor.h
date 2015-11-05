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


				enum ScanPatternType{
					FULL_FIELD_SCAN,
					BOUNDED_ELEVATION_SCAN,
					REGION_SCAN
				};

				
				
				//Constructor
				LaserSensor();

				//Destructor
				~LaserSensor();
				


				/* Renders the scene to Depth Texture using 'Render to Texture' feature 
				   of OpenGL. This function creates depth textures relative to the current
				   position of the sensor. If one has to increase the accuracy of the texture,
				   he would have to use a 16 bit depth buffer instead of 8 Bit. On the other hand
				   if someone has to increase the resolution of the sensor, then he would have
				   to increase the width and height of the texture on which the scene is rendering.
				   */
				void renderSceneToDepthTexture(GLuint &dfbo, 
												float lookAt[],
												float upDirection[],
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
				bool generateOffsetTextures(GLuint &dtexture_offset, 
											GLuint &dfbo_offset,
											int texture_width,
											int texture_height);


				
				bool generateRenderingDepthTextures(GLuint &dtexture, 
													GLuint &dfbo,
													int texture_width,
													int texture_height);



				/*
				 *	Creation of Vertex shader and fragment shader for rendering
				 */
				bool CreateShaders();


				/*
					Returns the Point Cloud for Full Field Scan Pattern
				*/
				void generateRE0xPointCloudFullScan(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points		
							);

				
				/*
					Returns the Point Cloud for Bounded Elevation Scan Pattern
				*/
				void generateRE0xPointCloudBoundedElevation(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points,
							float upperBound,
							float lowerBound
							);

				/*
					Returns the Point Cloud for Region Scan Pattern
				*/
				void generateRE0xPointCloudRegionScan(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points,
							float upperBound,
							float lowerBound,
							float angularRight,
							float angularLeft
							);


				
				/*
				Set the window size for which the Depth map needs to be generated.
				Sets the height and width of the depth map.
				*/
				void setSensorWindow(int height, int width){_width = width; _height = height;}
				
				/*
				Sets the position(x,y,z) of the sensor in 3D Space.
				*/
				void setSensorPosition(float xpos, float ypos, float zpos){_CameraPosition[0] = xpos; _CameraPosition[1] = ypos; _CameraPosition[2] = zpos;}
				
				/*
				Sets the position(x,y,z) of the sensor in 3D Space.
				*/
				void setSensorOrientation(float roll, float pitch, float yaw){_CameraOrientation[0] = roll; _CameraOrientation[1] = pitch; _CameraOrientation[2] = yaw;}
				

				/*
				Returns the position(x,y,z) of the sensor in 3D Space.
				*/
				float* getSensorPosition(){return _CameraPosition;}


				/*
				Returns the Orientation(roll,pitch,yaw) of the sensor in 3D Space.
				*/
				float* getSensorOrientation(){return _CameraOrientation;}



				void setSamplingFrequency(int freq){ _samplingFrequency = freq;}

				void startClock();

				void LaserSensor::multiplyRotationalMatrix(float *inVec, float *outVec);
				

			signals:
				//void sendData(QByteArray packetToBeSent);		
				void sendData(char* data, int length);		

			private:
			
				int _width;
				int _height;

				
				//should be private
				int _index;

				
				//scanLineAdd - Whether the sensor is moving from top to bottom
				//				or bottom to top				
				int _scanLineIndex, _scanLineAdd;

				
				//Laser sensor Position from where we are sensing the environment
				float _CameraPosition[3];

				//Laser Sensor Orientation - Roll, Pitch, Yaw
				float _CameraOrientation[3];
				
				
				//Total number of scanlines
				int _scanLines;
				
				//Frequency at which the sensor is sampling points
				int _samplingFrequency;

				//Shader IDs for the OpenGL rendering - should be private
				GLuint _ProgramId, _VertexShaderId, _FragmentShaderId;

				//Near Plane, Far Plane - don't know azimuth angle
				float _znear, _zfar;

				//frequency at which the sensor is rotating its head
				int _azimuthal_frequency;

			

				void performScan(ScanPatternType scanMode,
								GLuint final_depth_texture_fbo[],
								int texture_width, 
								int texture_height,
								float *points,
								float upperBound,
								float lowerBound,
								float angularRight,
								float angularLeft
								);

				


		};
	}
}

#endif