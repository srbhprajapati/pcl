#include <iostream>
#include <fstream>
#include <cstring>

#include <pcl/simulation/laser_sensor.h>
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

using namespace std;

GLuint VBO;
GLuint ProgramId, VertexShaderId, FragmentShaderId;
float f[] = {20.0, 20.0, 0.0, 1.0, 0.0, 0.0, 20.0, -20.0, 0.0, 0.0, 1.0, 0.0, -20.0, -20.0, 0.0, 0.0, 0.0, 1.0,
			-20.0, -20.0, 0.0, 1.0, 0.0, 0.0, -20.0, 20.0, 0.0, 0.0, 1.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 1.0};
				
void pcl::simulation::LaserSensor::initSensor(void)
{
    GLenum ErrorCheckValue = glGetError();
     
    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	ifstream ifs("J:/sourabh/Point Cloud Library/pcl-master/simulation/src/sensor.vert");
	std::string str( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );
	GLchar *cstr = new GLchar[str.size() + 1];
	strcpy(cstr, str.c_str());
	const GLchar *vertexShader = cstr;
	
	
	//cout<<"Vertex Shader : "<<endl;
	//printf("%s\n",cstr);
	
	glShaderSource(VertexShaderId, 1, &vertexShader, NULL);
    glCompileShader(VertexShaderId);
    FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
    ifstream ifsf("J:/sourabh/Point Cloud Library/pcl-master/simulation/src/sensor.frag");
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
   
	cout<<"VertexShaderId:"<<VertexShaderId<<endl;
	cout<<"FragmentShaderId:"<<FragmentShaderId<<endl;
	cout<<"PROGRAM ID:"<<ProgramId<<endl;
	
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

	const GLubyte* version = glGetString (GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;
  
  /*glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90, 4/3, 10.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  gluLookAt(0.0f, 0.0f, 15.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f);
*/
  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(f), f, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
	
}

void pcl::simulation::LaserSensor::generateData(void)
{

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
	
}