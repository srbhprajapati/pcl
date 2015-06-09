#ifndef LASERSENSORTHREAD_H
#define LASERSENSORTHREAD_H

#include <gl/glew.h>
#include <QGLWidget>
#include <QGLFunctions>
//Include PCL headers

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/PolygonMesh.h>
#include <pcl/simulation/camera.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/laser_sensor.h>

#include <cstring>
#include <cmath>

#include <QtOpenGL/QGLWidget>
#include <qmutex.h>
#include <qwaitcondition.h>
#include <QtNetwork/QUdpSocket>


#define PI 3.14159265

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;



#include <qgl.h>
#include <qthread.h>
#include <qsize.h>

class LaserGLWidget;

/** CPU time saving OpenGL rendering thread.
 * This thread only renders when its corresponding QGLWidget subclass
 * render() method is called (might be called from a different thread).
 * This class also provides a thread-safe example implementation of OpenGL object
 * picking capabilities (in the faceAtPosition() method).
 */
class LaserSensorThread : public QThread,  protected QGLFunctions
{
	Q_OBJECT
public:
    /** Init an OpenGl render thread for the _glwidget QGL */
    LaserSensorThread(LaserGLWidget& _glw);
	
    /** main() function of the thread. */
    void run();
	
    /** Signal the thread to exit the next time it is woken up. */
    void stop();
	
    /** Request a resize of the GL viewport.
     * This is usually called from the QGLWidgets resizeEvent() method.
     */
    void resizeViewport(const QSize& _size);

    // Sensor implementation
    /** Change settings for rendering. */
    void setRotation( GLfloat _x, GLfloat _y, GLfloat _z);
	
	
    /** Returns the color of a cube face.
     * This function can be called from different threads!
     */
    int faceAtPosition(const QPoint &pos);
	
	
    /** The six face colors of the cube
     * This should not be public!
     */
    QColor faceColors[6];



    /// \brief  Sends the Data through UDP Socket
    void sendData();

	void start_laser_sensor(int azm, int scan);
	
	void stop_laser_sensor();

	void changeScanMode(int mode, int Azimuthal, int Scanline, float upper_bound, float lower_bound, float lAngular, float rAngular_right);

	void changeSensorPosition(float x, float y, float z);
	
	void changeModel(QString path);

protected:
    
	/** Init the GL environment. */
    void initializeGL();
	
    /** Handles resizes of the GL viewport. */
    void resizeGL(int width, int height);
	
    /** Does all the painting. */
    void paintGL();

private:
    /** Actually draws the example scene (cube). */
    void draw();
	
    /** The QGLWidget of the render thread.
     * This widget provides the GL rendering context.
     */
    LaserGLWidget& glw;
    /** Keep the thread running as long this flag is true. */
    volatile bool render_flag;
    /** Perform a resize when this flag is true. */
    volatile bool resize_flag;
    /** Current size of the viewport. */
    QSize viewport_size;

    // example implmentation members
    GLfloat rotationX;
    GLfloat rotationY;
    GLfloat rotationZ;

	
	GLuint VBO;
	int texture_width;
	int texture_height;
	int window_width;
	int window_height;
	
	float u_bound, l_bound, angular_right, angular_left;

	void load_PolygonMesh_model (char* polygon_file);
	Scene::Ptr scene_ ;
	LaserSensor *ls1;
	GLuint depth_texture[4], depth_texture_1[4], fbo_[4] , fbo_1[4];
	
	char scanPatternType;

	float x;
//	float *f;
	float *points;

	bool doRendering;
	
	QUdpSocket *socket;
};


#endif // LASERSENSORTHREAD_H

