#ifndef _LASER_SENSOR_WRAPPER_H_
#define _LASER_SENSOR_WRAPPER_H_


#define TEXTURE_WIDTH 9000
#define TEXTURE_HEIGHT 9000
#define MAXIMUM_POINTS 100000


#include <gl/glew.h>
#include <QGLWidget>
#include <QGLFunctions>
#include <QtCore\qtimer.h>

#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/laser_sensor.h>
#include <pcl/simulation/laser_sensor_udp_interface.h>

#include <cstring>
#include <cmath>

namespace pcl
{
	namespace simulation
	{
		class PCL_EXPORTS LaserSensorWrapper : public QGLWidget, protected QGLFunctions
		{
			Q_OBJECT
			public:
				
				//constructor
				explicit LaserSensorWrapper(QWidget *parent = 0);
				
				//destructor
				~LaserSensorWrapper();

				
				enum ScanPatterns{
					FULL_SCAN,
					BOUNDED_ELEVATION_SCAN,
					REGION_SCAN
				};

				
			protected:
				
				//Initalizing the sensor
				void initializeGL();

				//Capturing points using the sensor
				void paintGL();

				//Resizing the current Rendering window
				void resizeGL(int width, int height);

			private slots:
				
				//Starts the sensor				
				void startSensor(int Sampling_Frequency);

				//Changes current Scan pattern to Full Field Scan
				void startFullFieldScan(int Azimuthal_value, int Scanline_value);

				//Changes current Scan pattern to Bounded Elevation 
				void startBoundedElevationScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound);

				//Changes current Scan pattern to Region Scan 
				void startRegionScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound, float lAngular, float rAngular);

				//Stops the sensor
				void stopSensor();

				//Changes the Model on which Scanning is performed
				void changeModel(QString path);

				
				//Saves the point Cloud
				void saveModel(QString path);

				//Changes position and orientation of the Camera				
				void changeSensorPose(float x, float y, float z, float roll, float pitch, float yaw);




				void update();

			private:
	
				//Loading OBJ and PLY model for simulating the scene
				bool load_PolygonMesh_model (std::string polygon_file);

				//multiply any vector to the current Rotational Matrix
				void multiplyRotationalMatrix(float *inVec, float *outVec);

				//Method for reGenerating the textures to refresh the scene. This method is
				//called in cases when the Sensor position or orientation is changed
				void reRenderScene();
				
				//Vertex Buffer Object for storing the vertices of two triangles i.e. rectangle(image) 
				//where the rendering needs to happen
				GLuint VBO;

				// Depth Texture Height and Width
				int texture_width, texture_height;

				//Rendering Window Width and Height
				int window_width, window_height;
	
				//Parameters of the sensor: Upper Bound, Lower Bound, Angular Left, Angular Right
				float u_bound, l_bound, angular_right, angular_left;
				int Azimuthal_freq, Number_of_Scanlines;

	
				//Scene on which we need the simulation to be performed on
				Scene::Ptr scene_ ;

				//Instance of RE0x Sensor
				LaserSensor *ls1;

				//Defining the textures on which the the scene needs to be rendered
				GLuint depth_texture, depth_texture_1[4], fbo_ , fbo_1[4];
	
				//Scan Pattern Type
				//char scanPatternType;

				ScanPatterns currentScanPattern;

				//All the points which are sensed by the sensor
				float *points;

				LaserSensorUdpInterface *_udp;

				bool _isSensorActive;

				QTimer *_renderTimer;

		};
	}
}

#endif //_LASER_SENSOR_WRAPPER_H_