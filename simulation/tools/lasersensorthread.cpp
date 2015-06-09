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
    faceColors[0] = "red";
    faceColors[1] = "green";
    faceColors[2] = "blue";
    faceColors[3] = "cyan";
    faceColors[4] = "yellow";
    faceColors[5] = "magenta";

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

	/*
	f[0]=1.0;		f[1]=1.0;		f[2]=0.0;		f[3]=1.0;		f[4]=1.0;
	f[5]=1.0;		f[6]=-1.0;		f[7]=0.0;		f[8]=1.0;		f[9]=0.0;
	f[10]= -1.0;	f[11]= -1.0;	f[12]=0.0;		f[13]=0.0;		f[14]=0.0;
	f[15]= -1.0;	f[16]= 1.0;		f[17]=0.0;		f[18]=0.0;		f[19]=1.0;*/

	
    socket = new QUdpSocket(this);
    socket->bind(QHostAddress::Any, 1234);

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
    render_flag = false;
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
			sendData();
			// swap the buffers of the GL widget
			glw.swapBuffers();
		}

        //glw.doneCurrent(); // release the GL render context to make picking work!

        //// wait until the gl widget says that there is something to render
        //// glwidget.lockGlContext() had to be called before (see top of the function)!
        //// this will release the render mutex until the wait condition is met
        //// and will lock the render mutex again before exiting
        //// waiting this way instead of insane looping will not waste any CPU ressources
        //glw.renderCondition().wait(&glw.renderMutex());

        //glw.makeCurrent(); // get the GL render context back

        // DEACTIVATED -- alternatively render a frame after a certain amount of time
        // prevent to much continous rendering activity
        // msleep(16); //sleep for 16 ms
    }
    // unlock the render mutex before exit
    glw.unlockGLContext();
}

void LaserSensorThread::initializeGL()
{
    // typical OpenGL init
    // see OpenGL documentation for an explanation
 /*   glw.qglClearColor("black");
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);*/

		
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
		/*		x += 0.9;
		glRotatef(x , 1.0, 1.0, 1.0);
*/


	
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

	
		//Sleep(800);
//		QGLWidget::swapBuffers();
	//	updateGL();
//		glutSwapBuffers();
//		glutPostRedisplay();

}

void LaserSensorThread::draw()
{
    // draws the cube
    static const GLfloat coords[6][4][3] =
        {
            {
                {
                    +1.0, -1.0, +1.0
                }
                , { +1.0, -1.0, -1.0 },
                { +1.0, +1.0, -1.0 }, { +1.0, +1.0, +1.0 }
            },
            { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, +1.0 },
              { -1.0, +1.0, +1.0 }, { -1.0, +1.0, -1.0 } },
            { { +1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 },
              { -1.0, +1.0, -1.0 }, { +1.0, +1.0, -1.0 } },
            { { -1.0, -1.0, +1.0 }, { +1.0, -1.0, +1.0 },
              { +1.0, +1.0, +1.0 }, { -1.0, +1.0, +1.0 } },
            { { -1.0, -1.0, -1.0 }, { +1.0, -1.0, -1.0 },
              { +1.0, -1.0, +1.0 }, { -1.0, -1.0, +1.0 } },
            { { -1.0, +1.0, +1.0 }, { +1.0, +1.0, +1.0 },
              { +1.0, +1.0, -1.0 }, { -1.0, +1.0, -1.0 } }
        };

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);
    glRotatef(rotationX, 1.0, 0.0, 0.0);
    glRotatef(rotationY, 0.0, 1.0, 0.0);
    glRotatef(rotationZ, 0.0, 0.0, 1.0);

    for (int i = 0; i < 6; ++i)
    {
        // assign names for each surface
        // this make picking work
        glLoadName(i);
        glBegin(GL_QUADS);
        glw.qglColor(faceColors[i]);
        for (int j = 0; j < 4; ++j)
        {
            glVertex3f(coords[i][j][0], coords[i][j][1],
                       coords[i][j][2]);
        }
        glEnd();
    }
}

int LaserSensorThread::faceAtPosition(const QPoint &pos)
{
    // we need to lock the rendering context
    glw.lockGLContext();

    // this is the same as in every OpenGL picking example
    const int MaxSize = 512; // see below for an explanation on the buffer content
    GLuint buffer[MaxSize];
    GLint viewport[4];

    glGetIntegerv(GL_VIEWPORT, viewport);
    glSelectBuffer(MaxSize, buffer);
    // enter select mode
    glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPickMatrix((GLdouble)pos.x(),
                  (GLdouble)(viewport[3] - pos.y()),
                  5.0, 5.0, viewport);
    GLfloat x = (GLfloat)viewport_size.width() / viewport_size.height();
    glFrustum(-x, x, -1.0, 1.0, 4.0, 15.0);
    draw();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();


    // finally release the rendering context again
    if (!glRenderMode(GL_RENDER))
    {
        glw.unlockGLContext();
        return -1;
    }
    glw.unlockGLContext();

    // Each hit takes 4 items in the buffer.
    // The first item is the number of names on the name stack when the hit occured.
    // The second item is the minimum z value of all the verticies that intersected
    // the viewing area at the time of the hit. The third item is the maximum z value
    // of all the vertices that intersected the viewing area at the time of the hit
    // and the last item is the content of the name stack at the time of the hit
    // (name of the object). We are only interested in the object name
    // (number of the surface).

    // return the name of the clicked surface
    return buffer[3];
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
//	render_flag = false;
	
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
	render_flag = false;
	ls1->CameraPosition[0] = x;
	ls1->CameraPosition[1] = y;
	ls1->CameraPosition[2] = z;
	render_flag = true;
}


void LaserSensorThread::sendData()
{
    QByteArray Data;

    for(int i=0; i<10000; i++)
    {
        Data.append("Data sent from OpenGL Thread \n");
    }
    int bytesWritten = socket->writeDatagram(Data, QHostAddress::LocalHost, 1234);

    qDebug()<<"Bytes Written : "<<bytesWritten;
}

