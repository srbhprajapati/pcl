#include <gl/glew.h>
#include "lasersensorthread.h"
#include "laserglwidget.h"

#include <QtOpenGL/QtOpenGL>
#include <gl/GLU.h>

LaserSensorThread::LaserSensorThread( LaserGLWidget & _glw )
        : QThread(),
        glw(_glw),
        render_flag(true),
        resize_flag(false),
        viewport_size(_glw.size()),
		QGLFunctions()

{
    // example implemenation init
    rotationX = 0;
    rotationY = 0;
    rotationZ = 0;

	texture_width = 1600;
	texture_height = 1600;

	window_width = 800;
	window_height = 630;

	u_bound= 0.0;
	l_bound= -30.0;
	angular_right=0.0;
	angular_left=0.0;

	//f = new float [20];
	points = new float [100000*3];
	scanPatternType = 'N';
	x=0.0;
	doRendering = false;


}

void LaserSensorThread::resizeViewport( const QSize& _size )
{
    // set size and flag to request resizing
    viewport_size = _size;
    resize_flag = true;
}

void LaserSensorThread::stop( )
{
    // set flag to request thread to exit
    // REMEMBER: The thread needs to be woken up once
    // after calling this method to actually exit!
    doRendering = false;
}

void LaserSensorThread::run( )
{
    // lock the render mutex of the Gl widget
    // and makes the rendering context of the glwidget current in this thread
    glw.lockGLContext();

    // general GL init
    initializeGL();

    // do as long as the flag is true
    while( render_flag )
    {
        // resize the GL viewport if requested
        if (resize_flag)
        {
            resizeGL(viewport_size.width(), viewport_size.height());
            resize_flag = false;
        }

        // render code goes here
		if(doRendering)
		{
			paintGL();
			// swap the buffers of the GL widget
			glw.swapBuffers();
		}

    }
    // unlock the render mutex before exit
    glw.unlockGLContext();
}

void LaserSensorThread::initializeGL()
{
		
float f[] =	{1.0, 1.0, 0.0, 1.0, 1.0, 
			 1.0, -1.0, 0.0, 1.0, 0.0, 
			 -1.0, -1.0, 0.0, 0.0, 0.0,
			 -1.0, 1.0, 0.0, 0.0, 1.0};
 	GLenum err = glewInit ();
	if (GLEW_OK != err)
	{
		std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
		exit (-1);
	}

	std::cout << "Statucameras: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;

	if (glewIsSupported ("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
	else
	{
		std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
		exit(1);
	}


	initializeGLFunctions();

	glEnable(GL_DEPTH_TEST);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-12.0f, 12.0f, -12.0f, 12.0f, 1.0f, 70.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(18.0f, 18.0f, 18.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 1.0f, 0.0f);

		glColor3f (1.0, 1.0, 1.0);

	scene_ = Scene::Ptr (new Scene ());
	ls1 = new LaserSensor(); 
	ls1->CreateShaders();

	
	load_PolygonMesh_model("E:/dataset/file.obj");
	

	ls1->generatetextures(depth_texture[0], depth_texture_1[0], fbo_[0], fbo_1[0], texture_width, texture_height);
	ls1->generatetextures(depth_texture[1], depth_texture_1[1], fbo_[1], fbo_1[1], texture_width, texture_height);
	ls1->generatetextures(depth_texture[2], depth_texture_1[2], fbo_[2], fbo_1[2], texture_width, texture_height);
	ls1->generatetextures(depth_texture[3], depth_texture_1[3], fbo_[3], fbo_1[3], texture_width, texture_height);
	cout<<"textures generated"<<endl;

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData (GL_ARRAY_BUFFER, sizeof(f), f, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];
	lookAt1[0] = ls1->CameraPosition[0] + 0.0; lookAt1[1] = ls1->CameraPosition[1] + 0.0; lookAt1[2] = ls1->CameraPosition[2] - 1.0;
	lookAt2[0] = ls1->CameraPosition[0] + 1.0; lookAt2[1] = ls1->CameraPosition[1] + 0.0; lookAt2[2] = ls1->CameraPosition[2] + 0.0;
	lookAt3[0] = ls1->CameraPosition[0] - 1.0; lookAt3[1] = ls1->CameraPosition[1] + 0.0; lookAt3[2] = ls1->CameraPosition[2] + 0.0;
	lookAt4[0] = ls1->CameraPosition[0] + 0.0; lookAt4[1] = ls1->CameraPosition[1] + 0.0; lookAt4[2] = ls1->CameraPosition[2] + 1.0;

	float CameraPosition[3];
	CameraPosition[0] = ls1->CameraPosition[0];
	CameraPosition[1] = ls1->CameraPosition[1];
	CameraPosition[2] = ls1->CameraPosition[2];


	ls1->renderSceneToDepthTexture(fbo_[0], lookAt1, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[1], lookAt2, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[2], lookAt3, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[3], lookAt4, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);

		
	ls1->depthTextureToRealDepthValues(depth_texture[0], fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[1], fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[2], fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[3], fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);
	

	cout<<"depth images generated"<<endl;


}

void LaserSensorThread::resizeGL(int width, int height)
{
    // nothing special
    // see OpenGL documentation for an explanation
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat x = (GLfloat)width / height;
    glFrustum(-x, x, -1.0, 1.0, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}

void LaserSensorThread::paintGL()
{
    //// clear all and draw the scene

	glClearColor (0.0, 0.0, 0.0, 0.0);	
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

	
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-12.0f, 12.0f, -12.0f, 12.0f, 1.0f, 70.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(18.0f, 18.0f, 18.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 1.0f, 0.0f);

		glColor3f (1.0, 1.0, 1.0);
		
		glRotatef(rotationX, 1.0, 0.0, 0.0);
		glRotatef(rotationY, 0.0, 1.0, 0.0);
		glRotatef(rotationZ, 0.0, 0.0, 1.0);
		
	
		float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];
		lookAt1[0] = ls1->CameraPosition[0] + 0.0; lookAt1[1] = ls1->CameraPosition[1] + 0.0; lookAt1[2] = ls1->CameraPosition[2] - 1.0;
		lookAt2[0] = ls1->CameraPosition[0] + 1.0; lookAt2[1] = ls1->CameraPosition[1] + 0.0; lookAt2[2] = ls1->CameraPosition[2] + 0.0;
		lookAt3[0] = ls1->CameraPosition[0] - 1.0; lookAt3[1] = ls1->CameraPosition[1] + 0.0; lookAt3[2] = ls1->CameraPosition[2] + 0.0;
		lookAt4[0] = ls1->CameraPosition[0] + 0.0; lookAt4[1] = ls1->CameraPosition[1] + 0.0; lookAt4[2] = ls1->CameraPosition[2] + 1.0;

		float CameraPosition[3];
		CameraPosition[0] = ls1->CameraPosition[0];
		CameraPosition[1] = ls1->CameraPosition[1];
		CameraPosition[2] = ls1->CameraPosition[2];


		ls1->renderSceneToDepthTexture(fbo_[0], lookAt1, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[1], lookAt2, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[2], lookAt3, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[3], lookAt4, CameraPosition, texture_width, texture_height, scene_, window_width, window_height);
	
		ls1->depthTextureToRealDepthValues(depth_texture[0], fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[1], fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[2], fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[3], fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);
	
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		switch (scanPatternType){
				case 'A':
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, texture_width, texture_height, CameraPosition, points);
					break;
				case 'B':
					ls1->generateRE0xPointCloudBoundedElevation(depth_texture_1, texture_width, texture_height, CameraPosition, points, u_bound, l_bound);
					break;
				case 'C':
					ls1->generateRE0xPointCloudRegionScan(depth_texture_1, texture_width, texture_height, CameraPosition, points, u_bound, l_bound, angular_right, angular_left);
					break;
				default:
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, texture_width, texture_height, CameraPosition, points);
					break;
		}

		glBegin(GL_POINTS);

		for(int i=0; i<100000; i++) 
			{
				glColor3f(points[3*i]*points[3*i], (points[3*i+1])*(points[3*i+1]), points[3*i+2]*points[3*i+2]);
				glVertex3f(points[3*i], points[3*i+1], points[3*i+2]);
			}
		glEnd();

		glColor3f(1.0f,0.0f,0.0f);


		glBegin(GL_QUADS);
			glVertex3f(CameraPosition[0]-0.1f, CameraPosition[1], CameraPosition[2]-0.1f);
			glVertex3f(CameraPosition[0]-0.1f, CameraPosition[1], CameraPosition[2]+0.1f);
			glVertex3f(CameraPosition[0]+0.1f, CameraPosition[1], CameraPosition[2]+0.1f);
			glVertex3f(CameraPosition[0]+0.1f, CameraPosition[1], CameraPosition[2]-0.1f);
		glEnd(); 

		glDisable(GL_DEPTH_TEST);
	


	glutSolidTeapot(2.0);

	
}

void LaserSensorThread::setRotation( GLfloat _x, GLfloat _y, GLfloat _z )
{
    rotationX += _x;
    rotationY += _y;
    rotationZ += _z;
}



void LaserSensorThread::load_PolygonMesh_model (char* polygon_file)
{
  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile ("E:/sourabh/ocular robotics/dataset/castle.obj", mesh); //City1_Block_1/new/City_new2.obj
  pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
  PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));

  scene_->add(model1);
  
  std::cout << "Just read file.obj"  << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
}


void LaserSensorThread::start_laser_sensor(int azm, int scan)
{
	
	std::cout<<"Azimuth : "<<azm<<std::endl;
	std::cout<<"ScanLine : "<<scan<<std::endl;

	ls1->azimuthal_frequency = azm;
	ls1->scanLines = scan;
	ls1->scanLineIndex=0;

	doRendering = true;

}

void LaserSensorThread::stop_laser_sensor()
{
	doRendering = false;
}


void LaserSensorThread::changeModel(QString path)
{
	doRendering = false;
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile (path.toStdString(), mesh);
  pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
  PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));

  scene_ = Scene::Ptr (new Scene ());
  scene_->add(model1);
  
  std::cout << "Just read file.obj"  << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";

  doRendering = true;
}

void LaserSensorThread::saveModel(QString path)
{
	doRendering = false;

	pcl::PointCloud<pcl::PointXYZ> cloud;

	for(int index=0; index<100000; index++)
	{
		cloud.push_back(pcl::PointXYZ(points[3*index], points[3*index+1], points[3*index+2]));
	}

	pcl::io::savePCDFileASCII(path.toStdString(), cloud);
	std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

	doRendering = true;
}


void LaserSensorThread::changeScanMode(int mode, int Azimuthal, int Scanline,  float upper_bound, float lower_bound, float lAngular, float rAngular)
{
	if(mode==0)
	{
		//Full Scan Mode
		
		ls1->azimuthal_frequency = Azimuthal;
		ls1->scanLines = Scanline;
		ls1->scanLineIndex=0;

		scanPatternType = 'A';
	}
	else if(mode==1)
	{
		//Bounded Elevation
		scanPatternType = 'B';
		u_bound = upper_bound;
		l_bound = lower_bound;
	}
	else if(mode==2)
	{
		//Region Scan Mode
		scanPatternType = 'C';
		u_bound = upper_bound;
		l_bound = lower_bound;
		angular_right = rAngular;
		angular_left = lAngular;
	}
	else
	{
		 //do Nothing
	}
}

void LaserSensorThread::changeSensorPosition(float x, float y, float z)
{
	doRendering = false;
	ls1->CameraPosition[0] = x;
	ls1->CameraPosition[1] = y;
	ls1->CameraPosition[2] = z;
	doRendering = true;
}


