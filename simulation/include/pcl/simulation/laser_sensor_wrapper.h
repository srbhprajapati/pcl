#ifndef _LASER_SENSOR_WRAPPER_H_
#define _LASER_SENSOR_WRAPPER_H_


#define TEXTURE_WIDTH 1600
#define TEXTURE_HEIGHT 1600
#define MAXIMUM_POINTS 100000


#include <gl/glew.h>
#include <QGLWidget>
#include <QGLFunctions>

#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/laser_sensor.h>

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
				
				
			protected:
				
				//Initalizing the sensor
				void initializeGL();

				//Capturing points using the sensor
				void paintGL();

				//Resizing the current Rendering window
				void resizeGL(int width, int height);


			private:
	
				//Loading OBJ and PLY model for simulating the scene
				void load_PolygonMesh_model (char* polygon_file);
	

				
				GLuint VBO;

				// Depth Texture Height and Width
				int texture_width, texture_height;

				//Rendering Window Width and Height
				int window_width, window_height;
	
				//Parameters of the sensor: Upper Bound, Lower Bound, Angular Left, Angular Right
				float u_bound, l_bound, angular_right, angular_left;

	
				//Scene on which we need the simulation to be performed on
				Scene::Ptr scene_ ;

				//Instance of RE0x Sensor
				LaserSensor *ls1;

				//Defining the textures on which the the scene needs to be rendered
				GLuint depth_texture[4], depth_texture_1[4], fbo_[4] , fbo_1[4];
	
				//Scan Pattern Type
				char scanPatternType;

				//All the points which are sensed by the sensor
				float *points;

		};
	}
}

#endif //_LASER_SENSOR_WRAPPER_H_