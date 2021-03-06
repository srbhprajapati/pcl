#include <pcl/simulation/laser_sensor_wrapper.h>
#include <iostream>
#include <QtWidgets>
#include <ctime>
#include <cmath>
#include <QtGui/qimage.h>

using namespace std;

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
	//scanPatternType = 'A';

	currentScanPattern = ScanPatterns::FULL_SCAN;


	_isSensorActive = true;

	ls1 = new LaserSensor(); 
	
	//Initialize the Scene Object
	scene_ = Scene::Ptr (new Scene ());
	
	_udp = new LaserSensorUdpInterface();
	_udp->initialize();

	//Connecting Signals and Slots
	QObject::connect(ls1,	SIGNAL(sendData(char*, int)),										_udp, SLOT(sendData(char*, int)));
	QObject::connect(_udp,	SIGNAL(startLaser(int)),											this, SLOT(startSensor(int)));
	QObject::connect(_udp,	SIGNAL(startFullFieldScan(int, int)),								this, SLOT(startFullFieldScan(int, int)));
	QObject::connect(_udp,	SIGNAL(startBoundedElevationScan(int, int, float, float)),			this, SLOT(startBoundedElevationScan(int, int, float, float)));
	QObject::connect(_udp,	SIGNAL(startRegionScan(int, int, float, float, float, float)),		this, SLOT(startRegionScan(int, int, float, float, float, float)));
	QObject::connect(_udp,	SIGNAL(stopLaser()),												this, SLOT(stopSensor()));
	QObject::connect(_udp,	SIGNAL(changeModel(QString)),										this, SLOT(changeModel(QString)));
	QObject::connect(_udp,	SIGNAL(saveModel(QString)),											this, SLOT(saveModel(QString)));
	QObject::connect(_udp,	SIGNAL(changeSensorPose(float, float, float, float, float, float)),	this, SLOT(changeSensorPose(float, float, float, float, float, float)));


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

	//Initialize GLEW
	GLenum err = glewInit ();
	if (GLEW_OK != err)
	{
		std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
		exit (-1);
	}
	std::cout << "Stas: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;


	//Check if OpenGL 2.0 is supported
	if (glewIsSupported ("GL_VERSION_2_0"))	std::cout << "OpenGL 2.0 supported" << std::endl;
	else
	{
		std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
		exit(1);
	}
						   
	//Initializes Qt OpenGL for current context
	initializeGLFunctions();


	//Set Orthographic Projection to view Console output
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



	//Create Shaders for the Laser Sensor Object
	std::cout<<"Creating Shaders... ";
	if(!ls1->CreateShaders())
	{
		std::cout<<"\nError In Creating Shaders... Exiting Application !!!"<<std::endl;
		exit(1);
	}
	else
	{
		std::cout<<"Done!"<<std::endl;
	}



	// Load Model for Sensing
	std::string fileName;
	std::cout<<"Enter Path of the Model : ";
	std::getline(std::cin, fileName);

	if(fileName.empty())
	{
		std::cout<<"Model PathName Empty... Exiting Application !!!"<<std::endl;
		exit(1);
		//fileName = "E:/sourabh/ocular robotics/dataset/castle.obj";
	}
	else
	{
		//replace back Slash with front slash
		std::replace(fileName.begin(), fileName.end(), '\\', '/');

		std::cout<<"Loading Model File... ";
		if(	!load_PolygonMesh_model(fileName))
		{
			std::cout<<"\nError In Loading Model... Exiting Application !!!"<<std::endl;
			exit(1);
		}
		else
		{
			std::cout<<"Done!"<<std::endl;
		}

	}
	
	
	//For Texture Allocation on GPU with low memory. Least supported texture will be 100 x 100
	while(texture_width>100)
	{
		
		bool _isAllTexturesAllocated = true;

		//Generate Rendering textures
		std::cout<<"Generating Rendering Texture... ";
		if(!ls1->generateRenderingDepthTextures(depth_texture, fbo_, texture_width, texture_height)) _isAllTexturesAllocated = false;

		
		//Generate Offset Textures
		std::cout<<"Generating Offset Textures... ";
		if(!ls1->generateOffsetTextures(depth_texture_1[0], fbo_1[0], texture_width, texture_height)) _isAllTexturesAllocated = false;
		if(!ls1->generateOffsetTextures(depth_texture_1[1], fbo_1[1], texture_width, texture_height)) _isAllTexturesAllocated = false;
		if(!ls1->generateOffsetTextures(depth_texture_1[2], fbo_1[2], texture_width, texture_height)) _isAllTexturesAllocated = false;
		if(!ls1->generateOffsetTextures(depth_texture_1[3], fbo_1[3], texture_width, texture_height)) _isAllTexturesAllocated = false;

		if(_isAllTexturesAllocated) break;
		else
		{
			std::cout<<"\nError In Generating Render/Offset Textures... Reducing Texture Resolution !!!"<<std::endl;

			glDeleteTextures(1, &depth_texture);
			glDeleteTextures(1, &depth_texture_1[0]);
			glDeleteTextures(1, &depth_texture_1[1]);
			glDeleteTextures(1, &depth_texture_1[2]);
			glDeleteTextures(1, &depth_texture_1[3]);

			glDeleteFramebuffers(1, &fbo_);
			glDeleteFramebuffers(1, &fbo_1[0]);
			glDeleteFramebuffers(1, &fbo_1[1]);
			glDeleteFramebuffers(1, &fbo_1[2]);
			glDeleteFramebuffers(1, &fbo_1[3]);

			texture_width = texture_width/2;
			texture_height = texture_height/2;

			std::cout<<"Trying to Allocate memory for Texture Width : "<<texture_width<<" and Texture_Height : "<<texture_height<<std::endl;
		}

	}

	//If the application is not allocate any memory to the minimum size textures
	if(texture_width<=100)
	{
		std::cout<<"GPU Memory too low for allocation to the textures...Exiting Application!!!"<<std::endl;
		exit(1);
	}
	else
	{
		std::cout<<"Done!"<<std::endl;
	}


	/*
	if(!ls1->generateOffsetTextures(depth_texture_1[0], fbo_1[0], texture_width, texture_height))
	{
			std::cout<<"\nError In Generating Offset Textures... Exiting Application !!!"<<std::endl;
			exit(1);
	}
	if(!ls1->generateOffsetTextures(depth_texture_1[1], fbo_1[1], texture_width, texture_height))
	{
			std::cout<<"\nError In Generating Offset Textures... Exiting Application !!!"<<std::endl;
			exit(1);
	}
	if(!ls1->generateOffsetTextures(depth_texture_1[2], fbo_1[2], texture_width, texture_height))
	{
			std::cout<<"\nError In Generating Offset Textures... Exiting Application !!!"<<std::endl;
			exit(1);
	}
	if(!ls1->generateOffsetTextures(depth_texture_1[3], fbo_1[3], texture_width, texture_height))
	{
			std::cout<<"\nError In Generating Offset Textures... Exiting Application !!!"<<std::endl;
			exit(1);
	}
	std::cout<<"Done!"<<std::endl;

	*/

	//Vertex Data for Two Triangles i.e. Image rectangle
	float f[] =	{1.0, 1.0, 0.0, 1.0, 1.0, 
				 1.0, -1.0, 0.0, 1.0, 0.0, 
				 -1.0, -1.0, 0.0, 0.0, 0.0,
				 -1.0, 1.0, 0.0, 0.0, 1.0};
 	
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData (GL_ARRAY_BUFFER, sizeof(f), f, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	float* SensorPosition = ls1->getSensorPosition();
	float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];															  

	float dir1[3], rotatedDir1[3];
	dir1[0] = 0.0; dir1[1] = 0.0; dir1[2] = -1.0; 
	multiplyRotationalMatrix(&dir1[0], &rotatedDir1[0]);
	lookAt1[0] = SensorPosition[0] + rotatedDir1[0]; lookAt1[1] = SensorPosition[1] + rotatedDir1[1]; lookAt1[2] = SensorPosition[2] + rotatedDir1[2];
															
	float dir2[3], rotatedDir2[3];
	dir2[0] = 1.0; dir2[1] = 0.0; dir2[2] = 0.0; 
	multiplyRotationalMatrix(&dir2[0], &rotatedDir2[0]);
	lookAt2[0] = SensorPosition[0] + rotatedDir2[0]; lookAt2[1] = SensorPosition[1] + rotatedDir2[1]; lookAt2[2] = SensorPosition[2] + rotatedDir2[2];

	float dir3[3], rotatedDir3[3];
	dir3[0] = -1.0; dir3[1] = 0.0; dir3[2] = 0.0; 
	multiplyRotationalMatrix(&dir3[0], &rotatedDir3[0]);
	lookAt3[0] = SensorPosition[0] + rotatedDir3[0]; lookAt3[1] = SensorPosition[1] + rotatedDir3[1]; lookAt3[2] = SensorPosition[2] + rotatedDir3[2];

	float dir4[3], rotatedDir4[3];
	dir4[0] = 0.0; dir4[1] = 0.0; dir4[2] = 1.0; 
	multiplyRotationalMatrix(&dir4[0], &rotatedDir4[0]);
	lookAt4[0] = SensorPosition[0] + rotatedDir4[0]; lookAt4[1] = SensorPosition[1] + rotatedDir4[1]; lookAt4[2] = SensorPosition[2] + rotatedDir4[2];


	float upDirection[3], rotatedUpDirection[3];	
	upDirection[0] = 0.0; upDirection[1] = 1.0; upDirection[2] = 0.0; 
	multiplyRotationalMatrix(&upDirection[0], &rotatedUpDirection[0]);


	ls1->renderSceneToDepthTexture(fbo_, lookAt1, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
	ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);

	ls1->renderSceneToDepthTexture(fbo_, lookAt2, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
	ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
	

	ls1->renderSceneToDepthTexture(fbo_, lookAt3, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
	ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
	
	ls1->renderSceneToDepthTexture(fbo_, lookAt4, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
	ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);

	//saveImageToFile(depth_texture_1[0]);
	//saveImageToFile(depth_texture_1[1]);
	//saveImageToFile(depth_texture_1[2]);
	//saveImageToFile(depth_texture_1[3]);

}


void LaserSensorWrapper::paintGL()
{

	//// clear all and draw the scene

	const clock_t begin_time = clock();
	// do something
	
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

		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
								  
		
		
		//If You want to Visualize the point Cloud on the console side
		
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


		
		swapBuffers();


		//glRotatef(0.1, 0.0, 1.0, 0.0);
		
		//updateGL();

		std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC <<std::endl;

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



bool LaserSensorWrapper::load_PolygonMesh_model (std::string polygon_file)
{
	//Check if File Exists
	ifstream file(polygon_file.c_str());
	if(!file.good())
	{
		file.close();
		return false;
	}
	file.close();

	//Load the File
	pcl::PolygonMesh mesh;	
	pcl::io::loadPolygonFile (polygon_file, mesh);
	pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
	PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));

	scene_->add(model1);
  
	std::cout <<"Polygons: "<< mesh.polygons.size () << " | Triangles: "<< mesh.cloud.data.size () <<" ... ";

	return true;
}


void LaserSensorWrapper::startSensor(int Sampling_Frequency)
{
	ls1->setSamplingFrequency(Sampling_Frequency);
	ls1->startClock();
	
	_renderTimer->start();
}


void LaserSensorWrapper::startFullFieldScan(int Azimuthal_value, int Scanline_value)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;

	//scanPatternType = 'A';

	currentScanPattern = ScanPatterns::FULL_SCAN;

}

void LaserSensorWrapper::startBoundedElevationScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;
	u_bound= upper_bound;
	l_bound= lower_bound;

	//scanPatternType = 'B';

	currentScanPattern = ScanPatterns::BOUNDED_ELEVATION_SCAN;
}

void LaserSensorWrapper::startRegionScan(int Azimuthal_value, int Scanline_value, float upper_bound, float lower_bound, float lAngular, float rAngular)
{
	Azimuthal_freq = Azimuthal_value; 
	Number_of_Scanlines	= Scanline_value;
	u_bound= upper_bound;
	l_bound= lower_bound;
	angular_right= rAngular;
	angular_left= lAngular;

	//scanPatternType = 'C';

	currentScanPattern = ScanPatterns::REGION_SCAN;
}

						   
void LaserSensorWrapper::stopSensor()
{
	_renderTimer->stop();
}													  

void LaserSensorWrapper::update()
{
	/*
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
		*/
		

	switch (currentScanPattern){

				case ScanPatterns::FULL_SCAN:
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points);
					break;

				case ScanPatterns::BOUNDED_ELEVATION_SCAN:
					ls1->generateRE0xPointCloudBoundedElevation(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points, u_bound, l_bound);
					break;

				case ScanPatterns::REGION_SCAN:
					ls1->generateRE0xPointCloudRegionScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points, u_bound, l_bound, angular_right, angular_left);
					break;

				default:
					ls1->generateRE0xPointCloudFullScan(depth_texture_1, Azimuthal_freq, Number_of_Scanlines, texture_width, texture_height, points);
					break;
		}

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


void LaserSensorWrapper::changeSensorPose(float x, float y, float z, float roll, float pitch, float yaw)
{
	ls1->setSensorPosition(x, y, z);
	ls1->setSensorOrientation(roll, pitch, yaw);
	reRenderScene();
}


void LaserSensorWrapper::reRenderScene()
{
		float* SensorPosition = ls1->getSensorPosition();
		float lookAt1[3], lookAt2[3], lookAt3[3], lookAt4[3];
		
		float dir1[3], rotatedDir1[3];
		dir1[0] = 0.0; dir1[1] = 0.0; dir1[2] = -1.0; 
		multiplyRotationalMatrix(&dir1[0], &rotatedDir1[0]);
		lookAt1[0] = SensorPosition[0] + rotatedDir1[0]; lookAt1[1] = SensorPosition[1] + rotatedDir1[1]; lookAt1[2] = SensorPosition[2] + rotatedDir1[2];
															
		float dir2[3], rotatedDir2[3];
		dir2[0] = 1.0; dir2[1] = 0.0; dir2[2] = 0.0; 
		multiplyRotationalMatrix(&dir2[0], &rotatedDir2[0]);
		lookAt2[0] = SensorPosition[0] + rotatedDir2[0]; lookAt2[1] = SensorPosition[1] + rotatedDir2[1]; lookAt2[2] = SensorPosition[2] + rotatedDir2[2];

		float dir3[3], rotatedDir3[3];
		dir3[0] = -1.0; dir3[1] = 0.0; dir3[2] = 0.0; 
		multiplyRotationalMatrix(&dir3[0], &rotatedDir3[0]);
		lookAt3[0] = SensorPosition[0] + rotatedDir3[0]; lookAt3[1] = SensorPosition[1] + rotatedDir3[1]; lookAt3[2] = SensorPosition[2] + rotatedDir3[2];

		float dir4[3], rotatedDir4[3];
		dir4[0] = 0.0; dir4[1] = 0.0; dir4[2] = 1.0; 
		multiplyRotationalMatrix(&dir4[0], &rotatedDir4[0]);
		lookAt4[0] = SensorPosition[0] + rotatedDir4[0]; lookAt4[1] = SensorPosition[1] + rotatedDir4[1]; lookAt4[2] = SensorPosition[2] + rotatedDir4[2];
																		 

		float upDirection[3], rotatedUpDirection[3];	
		upDirection[0] = 0.0; upDirection[1] = 1.0; upDirection[2] = 0.0; 
		multiplyRotationalMatrix(&upDirection[0], &rotatedUpDirection[0]);


		ls1->renderSceneToDepthTexture(fbo_, lookAt1, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
		ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[0], VBO, window_height, window_width, texture_width, texture_height);

		ls1->renderSceneToDepthTexture(fbo_, lookAt2, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
		ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[1], VBO, window_height, window_width, texture_width, texture_height);
	

		ls1->renderSceneToDepthTexture(fbo_, lookAt3, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
		ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[2], VBO, window_height, window_width, texture_width, texture_height);
	
		ls1->renderSceneToDepthTexture(fbo_, lookAt4, rotatedUpDirection, texture_width, texture_height, scene_, window_width, window_height);
		ls1->depthTextureToRealDepthValues(depth_texture, fbo_1[3], VBO, window_height, window_width, texture_width, texture_height);

		

}

void LaserSensorWrapper::multiplyRotationalMatrix(float *inVec, float *outVec)
{
	float rotMatrix[3][3];
	float* SensorOrientation = ls1->getSensorOrientation();

	float c3 =  cos(SensorOrientation[0]); //Roll
	float c2 =  cos(SensorOrientation[1]); //Pitch
	float c1 =  cos(SensorOrientation[2]); //Yaw

	float s3 =  sin(SensorOrientation[0]); //Roll
	float s2 =  sin(SensorOrientation[1]); //Pitch
	float s1 =  sin(SensorOrientation[2]); //Yaw

	rotMatrix[0][0] = c1*c2;
	rotMatrix[0][1] = c1*s2*s3 - c3*s1;
	rotMatrix[0][2] = s1*s3 + c1*s2*c3;
	rotMatrix[1][0] = c2*s1;
	rotMatrix[1][1] = c1*c3 + s1*s2*s3;
	rotMatrix[1][2] = c3*s1*s2 - c1*s3;
	rotMatrix[2][0] = -s2;
	rotMatrix[2][1] = c2*s3;
	rotMatrix[2][2] = c2*c3;

	outVec[0] = rotMatrix[0][0]*inVec[0] + rotMatrix[0][1]*inVec[1] + rotMatrix[0][2]*inVec[2];
	outVec[1] = rotMatrix[1][0]*inVec[0] + rotMatrix[1][1]*inVec[1] + rotMatrix[1][2]*inVec[2];
	outVec[2] = rotMatrix[2][0]*inVec[0] + rotMatrix[2][1]*inVec[1] + rotMatrix[2][2]*inVec[2];

}

void LaserSensorWrapper::saveImageToFile(GLuint textureID)
{
	std::cout<<"Texture Number : "<<textureID<<std::endl;

	glBindTexture(GL_TEXTURE_2D, textureID);

	unsigned char *pixelData = new unsigned char[texture_width*texture_width*4];
	glGetTexImage(GL_TEXTURE_2D, 0,GL_RGBA, GL_UNSIGNED_BYTE, pixelData);

	QImage *img = new QImage(pixelData, texture_width, texture_height, QImage::Format_RGB32);
	QString fileName = "Image" + QString::number(textureID) + ".jpg";

	img->save(fileName.toStdString().c_str(), "JPG", -1);

	img->~QImage();
	delete[] pixelData;

	glBindTexture(GL_TEXTURE_2D, 0);

}