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
	u_bound= 0.0;
	l_bound= -30.0;
	angular_right=0.0;
	angular_left=0.0;

	points = new float [MAXIMUM_POINTS*3];
	scanPatternType = 'N';

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
			glVertex3f(CameraPosition[0]-0.1f, CameraPosition[1], CameraPosition[2]-0.1f);
			glVertex3f(CameraPosition[0]-0.1f, CameraPosition[1], CameraPosition[2]+0.1f);
			glVertex3f(CameraPosition[0]+0.1f, CameraPosition[1], CameraPosition[2]+0.1f);
			glVertex3f(CameraPosition[0]+0.1f, CameraPosition[1], CameraPosition[2]-0.1f);
		glEnd(); 
*/
		glDisable(GL_DEPTH_TEST);
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
