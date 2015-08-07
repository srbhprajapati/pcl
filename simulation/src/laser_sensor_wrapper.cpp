#include <pcl/simulation/laser_sensor_wrapper.h>
#include <iostream>
#include <QtWidgets>

using namespace pcl::simulation;

LaserSensorWrapper::LaserSensorWrapper(QWidget *parent) :
    QGLWidget(parent)
{

	texture_width = TEXTURE_WIDTH;
	texture_height = TEXTURE_HEIGHT;

	window_width = 800;
	window_height = 630;

	//Initial Initalization - Can be anything
	Azimuthal_freq = 10; 
	Number_of_Scanlines	= 64;
	u_bound= 0.0;
	l_bound= -30.0;
	angular_right= 0.0;
	angular_left=40.0;

	points = new float [MAXIMUM_POINTS*3];
	scanPatternType = 'A';
	_isSensorActive = true;

	ls1 = new LaserSensor(); 

	_udp = new LaserSensorUdpInterface();
	_udp->initialize();

	//Connecting Signals and Slots
	/*QObject::connect(ls1, 
					 SIGNAL(sendData(QByteArray)), 
					 _udp, 
					 SLOT(sendData(QByteArray)));
	  */
	
	QObject::connect(ls1, 
					 SIGNAL(sendData(char*, int)), 
					 _udp, 
					 SLOT(sendData(char*, int)));


	QObject::connect(_udp, 
					 SIGNAL(startLaser(int, int)), 
					 this, 
					 SLOT(startSensor(int, int)));

	QObject::connect(_udp, 
					 SIGNAL(startFullFieldScan(int, int)), 
					 this, 
					 SLOT(startFullFieldScan(int, int)));

	QObject::connect(_udp, 
		             SIGNAL(startBoundedElevationScan(int, int, float, float)), 
					 this, 
					 SLOT(startBoundedElevationScan(int, int, float, float)));

	QObject::connect(_udp, 
					 SIGNAL(startRegionScan(int, int, float, float, float, float)), 
					 this, 
					 SLOT(startRegionScan(int, int, float, float, float, float)));

	QObject::connect(_udp, 
					 SIGNAL(stopLaser()), 
					 this, 
					 SLOT(stopSensor()));

	
	QObject::connect(_udp, 
					 SIGNAL(changeModel(QString)), 
					 this, 
					 SLOT(changeModel(QString)));

	
	QObject::connect(_udp, 
					 SIGNAL(saveModel(QString)), 
					 this, 
					 SLOT(saveModel(QString)));


	setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

    // Buffer swap is handled in the rendering thread
    setAutoBufferSwap(false);

	_renderTimer = new QTimer(this);
	_renderTimer->setInterval(50);

	QObject::connect(_renderTimer, SIGNAL(timeout()), this, SLOT(update()));
    
}

LaserSensorWrapper::~LaserSensorWrapper()
{
}


void LaserSensorWrapper::initializeGL()
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

    std::cout << "Stas: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;

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

	float* SensorPosition = ls1->getSensorPosition();
	float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];
	lookAt1[0] = SensorPosition[0] + 0.0; lookAt1[1] = SensorPosition[1] + 0.0; lookAt1[2] = SensorPosition[2] - 1.0;
	lookAt2[0] = SensorPosition[0] + 1.0; lookAt2[1] = SensorPosition[1] + 0.0; lookAt2[2] = SensorPosition[2] + 0.0;
	lookAt3[0] = SensorPosition[0] - 1.0; lookAt3[1] = SensorPosition[1] + 0.0; lookAt3[2] = SensorPosition[2] + 0.0;
	lookAt4[0] = SensorPosition[0] + 0.0; lookAt4[1] = SensorPosition[1] + 0.0; lookAt4[2] = SensorPosition[2] + 1.0;


	ls1->renderSceneToDepthTexture(fbo_[0], lookAt1, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[1], lookAt2, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[2], lookAt3, texture_width, texture_height, scene_, window_width, window_height);
	ls1->renderSceneToDepthTexture(fbo_[3], lookAt4, texture_width, texture_height, scene_, window_width, window_height);

		
	ls1->depthTextureToRealDepthValues(depth_texture[0], fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[1], fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[2], fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
	ls1->depthTextureToRealDepthValues(depth_texture[3], fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);
	

	cout<<"depth images generated"<<endl;

}


void LaserSensorWrapper::paintGL()
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
		
													 
		float* SensorPosition = ls1->getSensorPosition();
		float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];
		lookAt1[0] = SensorPosition[0] + 0.0; lookAt1[1] = SensorPosition[1] + 0.0; lookAt1[2] = SensorPosition[2] - 1.0;
		lookAt2[0] = SensorPosition[0] + 1.0; lookAt2[1] = SensorPosition[1] + 0.0; lookAt2[2] = SensorPosition[2] + 0.0;
		lookAt3[0] = SensorPosition[0] - 1.0; lookAt3[1] = SensorPosition[1] + 0.0; lookAt3[2] = SensorPosition[2] + 0.0;
		lookAt4[0] = SensorPosition[0] + 0.0; lookAt4[1] = SensorPosition[1] + 0.0; lookAt4[2] = SensorPosition[2] + 1.0;


		ls1->renderSceneToDepthTexture(fbo_[0], lookAt1, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[1], lookAt2, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[2], lookAt3, texture_width, texture_height, scene_, window_width, window_height);
		ls1->renderSceneToDepthTexture(fbo_[3], lookAt4, texture_width, texture_height, scene_, window_width, window_height);
	
		ls1->depthTextureToRealDepthValues(depth_texture[0], fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[1], fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[2], fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
		ls1->depthTextureToRealDepthValues(depth_texture[3], fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);
	
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		switch (scanPatternType){
				case 'A':
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points);
					break;
				case 'B':
					ls1->generateRE0xPointCloudBoundedElevation(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points, u_bound, l_bound);
					break;
				case 'C':
					ls1->generateRE0xPointCloudRegionScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points, u_bound, l_bound, angular_right, angular_left);
					break;
				default:
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points);
					break;
		}


		//If You want to Visualize the point Cloud on the console side
		/*
		glBegin(GL_POINTS);

		for(int i=0; i<100000; i++) 
			{
				glColor3f(points[3*i]*points[3*i], (points[3*i+1])*(points[3*i+1]), points[3*i+2]*points[3*i+2]);
				glVertex3f(points[3*i], points[3*i+1], points[3*i+2]);
			}
		glEnd();

		glColor3f(1.0f,0.0f,0.0f);


		glBegin(GL_QUADS);
            glVertex3f(SensorPosition[0]-0.1f, SensorPosition[1], SensorPosition[2]-0.1f);
            glVertex3f(SensorPosition[0]-0.1f, SensorPosition[1], SensorPosition[2]+0.1f);
            glVertex3f(SensorPosition[0]+0.1f, SensorPosition[1], SensorPosition[2]+0.1f);
            glVertex3f(SensorPosition[0]+0.1f, SensorPosition[1], SensorPosition[2]-0.1f);
		glEnd(); 

		glDisable(GL_DEPTH_TEST);
		  */
		//swapBuffers();

		//updateGL();

}

void LaserSensorWrapper::resizeGL(int width, int height)
{
	// change this
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat x = (GLfloat)width / height;
    glFrustum(-x, x, -1.0, 1.0, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}



void LaserSensorWrapper::load_PolygonMesh_model (char* polygon_file)
{
  pcl::PolygonMesh mesh;	
  pcl::io::loadPolygonFile ("E:/sourabh/ocular robotics/dataset/castle.obj", mesh);
  pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
  PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));

  scene_->add(model1);
  
  std::cout << "Just read file.obj"  << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
}


void LaserSensorWrapper::startSensor(int Sampling_Frequency)
{
	ls1->setSamplingFrequency(Sampling_Frequency);
	ls1->startClock();
	
}


void LaserSensorWrapper::startFullFieldScan(int Azimuthal_value, int Scanline_value)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;

	scanPatternType = 'A';
}

void LaserSensorWrapper::startBoundedElevationScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;
	u_bound= upper_bound;
	l_bound= lower_bound;

	scanPatternType = 'B';
}

void LaserSensorWrapper::startRegionScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound, float lAngular, float rAngular)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;
	u_bound= upper_bound;
	l_bound= lower_bound;
	angular_right= rAngular;
	angular_left= lAngular;

	scanPatternType = 'C';
}

						   
void LaserSensorWrapper::stopSensor()
{
	_renderTimer->stop();
}													  

void LaserSensorWrapper::update()
{
	paintGL();
}


void LaserSensorWrapper::changeModel(QString path)
{
	_renderTimer->stop();

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile (path.toStdString(), mesh);
  pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
  PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));

  scene_ = Scene::Ptr (new Scene ());
  scene_->add(model1);
  
  std::cout << "Just read file.obj"  << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";

  _renderTimer->start();
}


void LaserSensorWrapper::saveModel(QString path)
{
	  _renderTimer->stop();

	pcl::PointCloud<pcl::PointXYZ> cloud;

	for(int index=0; index<100000; index++)
	{
		cloud.push_back(pcl::PointXYZ(points[3*index], points[3*index+1], points[3*index+2]));
	}

	pcl::io::savePCDFileASCII(path.toStdString(), cloud);
	std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

	_renderTimer->start();
}