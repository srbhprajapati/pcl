/**
 * Demo program for sensor simulation 
 * A virtual sensor generates depth map for a scene.
 * Scene in the demo are generated in the Fragment Shader using Distance Field.
 * Two Shaders:
 *	Viewport 1: Display the scene for which the sensor data needs to be generated.
 *	Viewport 2: Sensor simulated depth map for the scene.
 */


#include <iostream>
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
#include <pcl/io/vtk_lib_io.h>"
#include <pcl/PolygonMesh.h>
#include <pcl/simulation/camera.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/laser_sensor.h>


using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;


//---------------Declare Global Objects------------
Scene::Ptr scene_ ;
Camera::Ptr camera_ ;
GLuint VBO;
GLuint ProgramId, VertexShaderId, FragmentShaderId;
LaserSensor sensor;


//---------------Two triangle Display(Scene to be defined in Fragment Shader)---------------------------------
float f[] = {20.0, 20.0, 0.0, 1.0, 0.0, 0.0, 20.0, -20.0, 0.0, 0.0, 1.0, 0.0, -20.0, -20.0, 0.0, 0.0, 0.0, 1.0,
			-20.0, -20.0, 0.0, 1.0, 0.0, 0.0, -20.0, 20.0, 0.0, 0.0, 1.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 1.0};



/*--------Reshape Routine for OpenGL Window-------*/
void display()
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	

	/*---------------------First Viewport: Display the scene-------------------------*/
	glViewport(0,0,640, 480);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90, 4/3, 10.0, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0f, 0.0f, 15.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f);

	glUseProgram(ProgramId);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	int index = glGetAttribLocation(ProgramId, "position");
	int col = glGetAttribLocation(ProgramId, "color");

	glEnableVertexAttribArray(index);
	glEnableVertexAttribArray(col);
	glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), 0);
	glVertexAttribPointer(col, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
	glDrawArrays(GL_TRIANGLES,0,6);
	glDisableVertexAttribArray(col);
	glDisableVertexAttribArray(index);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glUseProgram(0);
	

	/*------------Second Viewport: Display the sensor generated data (DEPTH MAP)----------------*/
	glViewport(640,0,640, 480);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90, 4/3, 10.0, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0f, 0.0f, 15.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f);

	sensor.generateData();

	glutSwapBuffers();
	glutPostRedisplay();
}

//// Read in a 3D model
//void load_PolygonMesh_model (char* polygon_file)
//{
//  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
//  //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
// /* pcl::io::loadPolygonFile ("J:/sourabh/Point Cloud Library/dataset/BrandenburgGate/BrandenburgGate.obj", mesh);
//  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));
//  
//  // Not sure if PolygonMesh assumes triangles if to
//  // TODO: Ask a developer
//  PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud));
//  scene_->add (model);
//  */
//  pcl::io::loadPolygonFile ("J:/sourabh/Point Cloud Library/dataset/file.obj", mesh);
//  pcl::PolygonMesh::Ptr cloud1 (new pcl::PolygonMesh (mesh));
//  
//  // Not sure if PolygonMesh assumes triangles if to
//  // TODO: Ask a developer
//  PolygonMeshModel::Ptr model1 = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud1));
//
//  scene_->add (model1);
//  
//  std::cout << "Just read file.obj"  << std::endl;
//  std::cout << mesh.polygons.size () << " polygons and "
//	    << mesh.cloud.data.size () << " triangles\n";
//  
//}


/*
This shader is for displaying the Scene(colour scene for reference).
TODO: Remove path dependency for Vertex and Fragment Shader.
*/
static void CreateShaders(void)
{
    GLenum ErrorCheckValue = glGetError();
     
    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	ifstream ifs("J:/sourabh/Point Cloud Library/pcl-master/simulation/tools/shader.vert");
	std::string str( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()) );
	GLchar *cstr = new GLchar[str.size() + 1];
	strcpy(cstr, str.c_str());
	const GLchar *vertexShader = cstr;
	
	//cout<<"Vertex Shader : "<<endl;
	//printf("%s\n",cstr);
	glShaderSource(VertexShaderId, 1, &vertexShader, NULL);
    glCompileShader(VertexShaderId);
    FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
    ifstream ifsf("J:/sourabh/Point Cloud Library/pcl-master/simulation/tools/shader.frag");
	std::string strf( (std::istreambuf_iterator<char>(ifsf) ),
                       (std::istreambuf_iterator<char>()    ) );
	GLchar *cstrf = new GLchar[strf.size() + 1];
	strcpy(cstrf, strf.c_str());
	const GLchar *fragmentShader = cstrf;

	//cout<<"Fragment Shader : "<<endl;
	//printf("%s\n",cstrf);
	glShaderSource(FragmentShaderId, 1, &fragmentShader, NULL);
    glCompileShader(FragmentShaderId);
    ProgramId = glCreateProgram();
        glAttachShader(ProgramId, VertexShaderId);
        glAttachShader(ProgramId, FragmentShaderId);
    glLinkProgram(ProgramId);
   // glUseProgram(ProgramId);
	cout<<"PROGRAM ID:"<<ProgramId<<" "<<VertexShaderId<<" "<<FragmentShaderId<<endl;
	GLint status;
	    glGetProgramiv (ProgramId, GL_LINK_STATUS, &status);
		cout<<status<<endl;
    if (status == GL_FALSE)
    {
        GLint infoLogLength;
        glGetProgramiv(ProgramId, GL_INFO_LOG_LENGTH, &infoLogLength);
        
        GLchar *strInfoLog = new GLchar[infoLogLength + 1];
        glGetProgramInfoLog(ProgramId, infoLogLength, NULL, strInfoLog);
        fprintf(stderr, "Linker failure: %s\n", strInfoLog);
        delete[] strInfoLog;
    }

	cout<<"error not reached"<<endl;
    ErrorCheckValue = glGetError();
    if (ErrorCheckValue != GL_NO_ERROR)
    {
        fprintf(
            stderr,
            "ERROR: Could not create the shaders: %s \n",
            gluErrorString(ErrorCheckValue)
        );
 
        exit(-1);
    }

	cout<<"ShaderCreated"<<endl;
}


/*---------
Main Function
----------*/

int
main (int argc, char** argv)
{
	int width = 1280;
	int height = 480;

	//--------------------------GLUT GLEW INITIALIZE----------------------------------
	glutInit (&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(width, height);
	glutCreateWindow ("OpenGL Sensor Simulation");

	GLenum err = glewInit ();
	if (GLEW_OK != err)
	{
		std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
		exit (-1);
	}

	std::cout << "Status: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;

	if (glewIsSupported ("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
	else
	{
		std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
		exit(1);
	}
	std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;



	//-----------Initialize PCL Objects (currently not in use)---------------
		camera_ = Camera::Ptr (new Camera ());
		scene_ = Scene::Ptr (new Scene ());
	


	//---------------------Initliaze Virtual Laser Sensor-------------------------
	sensor.initSensor();
	sensor.setSensorWindow(640, 480);
	sensor.printMessage();



	//---------Generate Scene for which Sensor data to be generated--------------
	CreateShaders();
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(f), f, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	

	//static const GLfloat ambient[4] = {0.5f, 0.1f, 0.1f, 0.1f};
	//static const GLfloat diffuse[4] = {0.5f, 1.0f, 1.0f, 1.0f};
	//static const GLfloat position0[4] = {0.0f, 0.0f, 15.0f, 0.0f};
	//static const GLfloat position1[4] = {0.0f, 0.0f, -200.0f, 0.0f};

	//
	//glDisable(GL_TEXTURE_2D);
	//glEnable(GL_DEPTH_TEST);

	////flShadeModel(GL)
	//glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_LIGHTING);
 //   glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
 //   glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
 //   glLightfv(GL_LIGHT0, GL_POSITION, position0);
 //   glEnable(GL_LIGHT0);
	//
 //   //glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
 //   //glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
 //   //glLightfv(GL_LIGHT1, GL_POSITION, position1);
 //   //glEnable(GL_LIGHT1);
	//glShadeModel( GL_SMOOTH );
	////glutReshapeFunc (on_reshape);
	glutDisplayFunc (display);
	glutIdleFunc (display);
	//glutKeyboardFunc (on_keyboard);
	//glutMouseFunc (on_mouse);
	//glutMotionFunc (on_motion);
	//glutPassiveMotionFunc (on_passive_motion);
	//glutEntryFunc (on_entry);
	glutMainLoop ();

	
	cout<<"Project simulation sensor started"<<endl;
	while(true){}
	return 0;
}