#include <iostream>
#include <fstream>
#include <cstring>
#include <QDebug>

#include <pcl/simulation/laser_sensor.h>

using namespace std;

GLuint VBO;
GLuint ProgramId, VertexShaderId, FragmentShaderId;
float f[] = {20.0, 20.0, 0.0, 1.0, 0.0, 0.0, 20.0, -20.0, 0.0, 0.0, 1.0, 0.0, -20.0, -20.0, 0.0, 0.0, 0.0, 1.0,
			-20.0, -20.0, 0.0, 1.0, 0.0, 0.0, -20.0, 20.0, 0.0, 0.0, 1.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0, 1.0};
				
void pcl::simulation::LaserSensor::initSensor(void)
{
    GLenum ErrorCheckValue = glGetError();
     
    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	ifstream ifs("C:/Users/Anubhav/Documents/GitHub/pcl/simulation/src/sensor.vert");
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
    ifstream ifsf("C:/Users/Anubhav/Documents/GitHub/pcl/simulation/src/sensor.frag");
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



// create texture
// display to framebuffer
// use program and texture to create real z data
//actually display the result

void  pcl::simulation::LaserSensor::renderSceneToDepthTexture(GLuint &dfbo, 
																float lookAt[],
																float CameraPosition[],
																int texture_width, 
																int texture_height,
																Scene::Ptr scene_,
																int window_width,
																int window_height)
{
	//-----------------------Initial Depth texture-------------------------------
	glBindFramebuffer (GL_FRAMEBUFFER, dfbo);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f (1.0, 1.0, 1.0);
	glViewport(0,0, (float)texture_width, (float)texture_height);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glFrustum(-1.0f, 1.0f, -1.0f, 1.0f, znear, zfar);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(CameraPosition[0], CameraPosition[1], CameraPosition[2],
			lookAt[0], lookAt[1], lookAt[2],
			0.0f, 1.0f, 0.0f);
	scene_->draw();
	glBegin(GL_QUADS);
		glVertex3f(10000.0f, 1.0f, 10000.0f);
		glVertex3f(-10000.0f, 1.0f, 10000.0f);
		glVertex3f(-10000.0f, 1.0f, -10000.0f);
		glVertex3f(10000.0f, 1.0f, -10000.0f);
	glEnd(); 
	glBindFramebuffer (GL_FRAMEBUFFER, 0);
    //-----------------------Initial Depth texture End------------------------------
	
	glViewport(0,0, (float)window_width, (float)window_height);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
}



void pcl::simulation::LaserSensor::depthTextureToRealDepthValues(GLuint &dtexture, 
									GLuint &dfbo_1,
									GLuint VBO,
									int window_height,
									int window_width,
									int texture_width,
									int texture_height)
{
	//-----------------------Depth texture Display-------------------------------
		glBindFramebuffer (GL_FRAMEBUFFER, dfbo_1);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColor3f (1.0, 1.0, 1.0);
		glViewport(0,0,texture_width, texture_height);
		glUseProgram(ProgramId);

		GLuint location_znear = glGetUniformLocation(ProgramId, "zNear");
		GLuint location_zfar = glGetUniformLocation(ProgramId, "zFar");
		glUniform1f(location_znear, znear);
		glUniform1f(location_zfar, zfar);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		gluLookAt(0.0f, 0.0f, 0.5f,
				0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f);
	

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
        int index1=glGetAttribLocation(ProgramId,"position");
        int index2=glGetAttribLocation(ProgramId,"texelvalue");
        glBindTexture(GL_TEXTURE_2D, dtexture);
		
        glUniform1i(glGetUniformLocation(ProgramId,"depth_texture"),0);
       
        glEnableVertexAttribArray(index1);
        glVertexAttribPointer(index1,3,GL_FLOAT,GL_FALSE,5*sizeof(float),0);
       
        glEnableVertexAttribArray(index2);
        glVertexAttribPointer(index2,2,GL_FLOAT,GL_FALSE,5*sizeof(float),(void*)(3*sizeof(float)));
       
        glDrawArrays(GL_QUADS,0,4);

		glDisableVertexAttribArray(index1);
		glDisableVertexAttribArray(index2);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);
		glUseProgram(0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
//-----------------------Depth texture Display End-------------------------------
	
		glViewport(0,0, (float)window_width, (float)window_height);
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	

}


void pcl::simulation::LaserSensor::generatetextures(GLuint &dtexture, 
						GLuint &dtexture_1, 
						GLuint &dfbo, 
						GLuint &dfbo_1,
						int texture_width,
						int texture_height)
{
	
	//generate texture for storage of depth image
	glGenTextures(1, &dtexture);
	glBindTexture(GL_TEXTURE_2D, dtexture);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, texture_width, texture_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glBindTexture (GL_TEXTURE_2D, 0);

	//generate framebuffer to attach to the depth texture
	glGenFramebuffers (1, &dfbo);
	glBindFramebuffer (GL_FRAMEBUFFER, dfbo);
	glFramebufferTexture2D (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dtexture, 0);
	
	
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if(status!= GL_FRAMEBUFFER_COMPLETE) cout<<"framebuffer incomplete"<<endl;
	glBindFramebuffer (GL_FRAMEBUFFER, 0);


	//create offset textures
	glGenTextures(1, &dtexture_1);
	glBindTexture(GL_TEXTURE_2D, dtexture_1);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, texture_width, texture_height, 0, GL_RGB, GL_FLOAT, NULL); // add offset data to offset buffer
	glBindTexture (GL_TEXTURE_2D, 0);


	//generate framebuffer to attach to the depth texture
	glGenFramebuffers (1, &dfbo_1);
	glBindFramebuffer (GL_FRAMEBUFFER, dfbo_1);
	glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dtexture_1, 0);
	
	GLenum status_1 = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if(status_1!= GL_FRAMEBUFFER_COMPLETE) cout<<"framebuffer incomplete"<<endl;
	glBindFramebuffer (GL_FRAMEBUFFER, 0);

	
}


/*
This shader is for displaying the Scene(colour scene for reference).
TODO: Remove path dependency for Vertex and Fragment Shader.
*/
void pcl::simulation::LaserSensor::CreateShaders()
{
    GLenum ErrorCheckValue = glGetError();
     
    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	ifstream ifs("E:/sourabh/ocular robotics/pcl/simulation/tools/shader.vert");
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
    ifstream ifsf("E:/sourabh/ocular robotics/pcl/simulation/tools/shader.frag");
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

void pcl::simulation::LaserSensor::getPointCloud(GLuint &dtexture_1, 
					int index,
					int texture_width,
					int texture_height,
					float CameraPosition[],
					float *points)
{
		//generate point cloud from the depth image texture
		//total points  800 x 800
		
	
	//get image from the texture
	GLint t_width, t_height, internalFormat;
	glBindTexture(GL_TEXTURE_2D, dtexture_1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &t_width);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &t_height);
	
	//cout<<"GL_DEPTH_COMPONENT : "<<internalFormat<<endl;
	GLuint numBytes=0;
	switch(internalFormat)
	{
		case GL_RGB:
			numBytes = t_width*t_height*3;
			break;
		case GL_RGBA:
			numBytes = t_width*t_height*4;
			break;
		case GL_DEPTH_COMPONENT:
			numBytes = t_width*t_height;
			break;
		default:
			break;
	}

	if(numBytes)
	{
		unsigned char *pixels = new unsigned char[numBytes];
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat, GL_UNSIGNED_BYTE, pixels);

		for(int i=0; i<64; i++)
		{
			
			//thetha = -44.8 + 1.4*x
			//phi = -45 + 0.45*y

			float thetha = 90 - (-44.8 + i*1.4);
			float cos_thetha = cos(thetha*PI/180.0);
			float sin_thetha = sin(thetha*PI/180.0);

			
			//int height_value = 4 + 3*i;
			

			for(int j=0; j<t_width; j++)
			{
				float phi = -45.0 + j*((float)45/(float)(texture_width/2));
				float cos_phi = cos(phi*PI/180.0);
				float sin_phi = sin(phi*PI/180.0);

				
				float height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
				float width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
				//convert distance from the camera to points in 3d space
				float direction[3];		//direction b/w pixel pos and camera pos
				

				if(index==0)
				{
					direction[0] = (2*(float)width_value/(float)texture_width - 1.0);
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = -1.0f;
				}
				else if(index == 1)
				{
					direction[0] = 1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (2*(float)width_value/(float)texture_width-1.0f);
				}
				else if(index==2)
				{
					direction[0] = -1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (1.0f - 2*(float)width_value/(float)texture_width);
				}
				else if(index==3)
				{
					direction[0] = (1.0 - 2*(float)width_value/(float)texture_width);
					direction[1] = (2*(float)height_value/(float)texture_height - 1.0);
					direction[2] = 1.0f;
				}

				float magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

				direction[0] /= magnitude;
				direction[1] /= magnitude;
				direction[2] /= magnitude;

				float distance = (float)pixels[3*(int)height_value*t_width + 3*(int)width_value]*zfar/ 255.0;

				float point[3];


				if(distance<(zfar-0.0001f))
				{
				point[0] = CameraPosition[0] + distance*direction[0];
				point[1] = CameraPosition[1] + distance*direction[1];
				point[2] = CameraPosition[2] + distance*direction[2];
				}
				else
				{
					point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;
				}
				
				//glVertex3f(point[0], point[1], point[2]);
				/*if(index==0)
				{*/
				points[3*i*t_width + 3*j] = point[0];
				points[3*i*t_width + 3*j + 1] = point[1];
				points[3*i*t_width + 3*j + 2] = point[2];
				/*}
				else if(index==1)
				{
				points1[3*i*t_width + 3*j] = point[0];
				points1[3*i*t_width + 3*j + 1] = point[1];
				points1[3*i*t_width + 3*j + 2] = point[2];
				}
				else if(index==2)
				{
				points2[3*i*t_width + 3*j] = point[0];
				points2[3*i*t_width + 3*j + 1] = point[1];
				points2[3*i*t_width + 3*j + 2] = point[2];
				}*/
			//	if((i*t_width + j)<6) 
				//cout<<"x = "<<point[0]<<" y = "<<point[1]<<" z = "<<point[2]<<endl;
			}
		}
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	//cout<<"numBytes : "<<numBytes<<endl;
	//cout<<"getPointCloud executed"<<endl;
}



void pcl::simulation::LaserSensor::generateRE0xPointCloudFullScan(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points		
							)
{
	//Get all the Textures 
	// Texture 1: -45 to 45 degree
	// Texture 2: 45 to 135 degree
	// Texture 3: 135 to 225 degree
	// Texture 4: 225 to 315 degree
	GLint t_width1, t_height1, internalFormat1;
	glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &t_width1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &t_height1);
	
	//cout<<"GL_DEPTH_COMPONENT : "<<internalFormat1<<endl;
	GLuint numBytes=0;
	switch(internalFormat1)
	{
		case GL_RGB:
			numBytes = t_width1*t_height1*3;
			break;
		case GL_RGBA:
			numBytes = t_width1*t_height1*4;
			break;
		case GL_DEPTH_COMPONENT:
			numBytes = t_width1*t_height1;
			break;
		default:
			break;
	}

	if(numBytes)
	{
		unsigned char *pixels = new unsigned char[numBytes];
		unsigned char *pixels2 = new unsigned char[numBytes];
		unsigned char *pixels3 = new unsigned char[numBytes];
		unsigned char *pixels4 = new unsigned char[numBytes];

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[1]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels2);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[2]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels3);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[3]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels4);


		//local Variables
		 
		int points_per_scanline = samplingFrequency/azimuthal_frequency;
		float scan_start_elevation, scan_end_elevation;
		float elevation_range_per_scanline = 90.0/(float)scanLines;
		float distance = 10.0;
		float angular_offset= 0.0, elevation_offset = 0.0, elevation_angle = 0.0, azimuthal_angle = 0.0, height_value, width_value;
		float thetha = 0.0, cos_thetha = 0.0, sin_thetha = 0.0, phi = 0.0, cos_phi =0.0, sin_phi =0.0;
		float direction[3], magnitude, point[3];


		// To copy the data into a Byte Array for transfer over UDP
		QByteArray UdpSensorData;
	    QDataStream dStream(&UdpSensorData, QIODevice::WriteOnly);
		int udpPointCounter = 0;

		
//		for(int i=0; i<azimuthal_frequency; i++)
//		{
			scan_start_elevation = 45.0 - (elevation_range_per_scanline * scanLineIndex);		//scan_elevation = from top the starting elevation for spiral scan
			scan_end_elevation = 45.0 - (elevation_range_per_scanline * (scanLineIndex+1));

			for(int j=0; j<points_per_scanline; j++)
			{
				angular_offset = 360.0*((float)j/(float)points_per_scanline);
				elevation_offset = elevation_range_per_scanline*((float)j/(float)points_per_scanline);

				if(scanLineAdd == 1)
				elevation_angle = scan_start_elevation - elevation_offset;	//moving from 45 to -45
				else elevation_angle = scan_end_elevation + elevation_offset;
				azimuthal_angle = angular_offset;
				
				thetha = 90.0 - elevation_angle;
				cos_thetha = cos(thetha*PI/180.0);
				sin_thetha = sin(thetha*PI/180.0);
				
				phi = azimuthal_angle;
				cos_phi = cos(phi*PI/180.0);
				sin_phi = sin(phi*PI/180.0);
				direction[3];		//direction b/w pixel pos and camera pos
				height_value, width_value ;

				if(azimuthal_angle>=45.0 && azimuthal_angle<135.0)				//texture 1
				{
					phi =  azimuthal_angle - 90;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (2*(float)width_value/(float)texture_width - 1.0);
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = -1.0f;

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}

				else if (azimuthal_angle>=135.0 && azimuthal_angle<225.0)		//texture 3
				{
					phi = azimuthal_angle - 180;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);
					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);

					direction[0] = -1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (1.0f - 2*(float)width_value/(float)texture_width);

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels3[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else if (azimuthal_angle>=225.0 && azimuthal_angle<315.0)		//texture 4
				{
					phi = azimuthal_angle - 270;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (1.0 - 2*(float)width_value/(float)texture_width);
					direction[1] = (2*(float)height_value/(float)texture_height - 1.0);
					direction[2] = 1.0f;
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels4[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else
				{
					if(azimuthal_angle>=315.0)
					{
						phi = azimuthal_angle - 360;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}
					else if(azimuthal_angle>=0.0 && azimuthal_angle<45.0)
					{
						phi= azimuthal_angle;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = 1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (2*(float)width_value/(float)texture_width-1.0f);
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels2[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;


				}
				
				
				if(distance<(zfar-0.0001f))
				{
				point[0] = CameraPosition[0] + distance*direction[0];
				point[1] = CameraPosition[1] + distance*direction[1];
				point[2] = CameraPosition[2] + distance*direction[2];
				}
				else
				{
					point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;
				}
				
				
				if(index>=100000)
				{
					index=0;
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];

				}
				else{
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];
				}
				index++;		

				
				if(udpPointCounter<100)
				{
					dStream<<point[0];
					dStream<<point[1];
					dStream<<point[2];	
					udpPointCounter++;
				}
				else
				{
					sendData(UdpSensorData);
					UdpSensorData.clear();
					dStream.device()->reset();
					udpPointCounter = 0;
				}
				  
				//std::cout<<"point "<<index<<": "<<point[0]<< " "<< point[1]<<" "<<point[2]<<std::endl;
			
			}
			if(scanLineIndex>=scanLines-1)
			{
					scanLineAdd = -1 ;
			}
			else if(scanLineIndex<=0) 
			{
					scanLineAdd = 1;
			}
			scanLineIndex+=scanLineAdd;

			if(udpPointCounter!=0)
			{
				sendData(UdpSensorData);
				UdpSensorData.clear();
				dStream.device()->reset();
				udpPointCounter = 0;
			}

		delete [] pixels;
		delete [] pixels2;
		delete [] pixels3;
		delete [] pixels4;

		//Sleep(1000/azimuthal_frequency);
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	//cout<<"numBytes : "<<numBytes<<endl;
	//cout<<"getPointCloud executed"<<endl;

}




void pcl::simulation::LaserSensor::generateRE0xPointCloudBoundedElevation(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points,
							float upperBound,
							float lowerBound
							)
{
	//Get all the Textures 
	// Texture 1: -45 to 45 degree
	// Texture 2: 45 to 135 degree
	// Texture 3: 135 to 225 degree
	// Texture 4: 225 to 315 degree
	GLint t_width1, t_height1, internalFormat1;
	glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &t_width1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &t_height1);
	
	cout<<"GL_DEPTH_COMPONENT : "<<internalFormat1<<endl;
	GLuint numBytes=0;
	switch(internalFormat1)
	{
		case GL_RGB:
			numBytes = t_width1*t_height1*3;
			break;
		case GL_RGBA:
			numBytes = t_width1*t_height1*4;
			break;
		case GL_DEPTH_COMPONENT:
			numBytes = t_width1*t_height1;
			break;
		default:
			break;
	}

	if(numBytes)
	{
		unsigned char *pixels = new unsigned char[numBytes];
		unsigned char *pixels2 = new unsigned char[numBytes];
		unsigned char *pixels3 = new unsigned char[numBytes];
		unsigned char *pixels4 = new unsigned char[numBytes];

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[1]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels2);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[2]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels3);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[3]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels4);


		//local Variables
		 
		int points_per_scanline = samplingFrequency/azimuthal_frequency;
		float scan_start_elevation, scan_end_elevation;
		float elevation_range_per_scanline = (upperBound - lowerBound)/(float)scanLines;
		float distance = 10.0;
		float angular_offset= 0.0, elevation_offset = 0.0, elevation_angle = 0.0, azimuthal_angle = 0.0, height_value, width_value;
		float thetha = 0.0, cos_thetha = 0.0, sin_thetha = 0.0, phi = 0.0, cos_phi =0.0, sin_phi =0.0;
		float direction[3], magnitude, point[3];

		
		
		// To copy the data into a Byte Array for transfer over UDP
		QByteArray UdpSensorData;
	    QDataStream dStream(&UdpSensorData, QIODevice::WriteOnly);
		int udpPointCounter = 0;

		//for(int i=0; i<azimuthal_frequency; i++)
		//{
			scan_start_elevation = upperBound - (elevation_range_per_scanline * scanLineIndex);		//scan_elevation = from top the starting elevation for spiral scan
			scan_end_elevation = upperBound - (elevation_range_per_scanline * (scanLineIndex+1));

			for(int j=0; j<points_per_scanline; j++)
			{
				angular_offset = 360.0*((float)j/(float)points_per_scanline);
				elevation_offset = elevation_range_per_scanline*((float)j/(float)points_per_scanline);

				if(scanLineAdd == 1)
				elevation_angle = scan_start_elevation - elevation_offset;	//moving from 45 to -45
				else elevation_angle = scan_end_elevation + elevation_offset;


				azimuthal_angle = angular_offset;
				
				thetha = 90.0 - elevation_angle;
				cos_thetha = cos(thetha*PI/180.0);
				sin_thetha = sin(thetha*PI/180.0);
				
				phi = azimuthal_angle;
				cos_phi = cos(phi*PI/180.0);
				sin_phi = sin(phi*PI/180.0);
				direction[3];		//direction b/w pixel pos and camera pos
				height_value, width_value ;

				if(azimuthal_angle>=45.0 && azimuthal_angle<135.0)				//texture 1
				{
					phi =  azimuthal_angle - 90;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (2*(float)width_value/(float)texture_width - 1.0);
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = -1.0f;

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}

				else if (azimuthal_angle>=135.0 && azimuthal_angle<225.0)		//texture 3
				{
					phi = azimuthal_angle - 180;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);
					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);

					direction[0] = -1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (1.0f - 2*(float)width_value/(float)texture_width);

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels3[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else if (azimuthal_angle>=225.0 && azimuthal_angle<315.0)		//texture 4
				{
					phi = azimuthal_angle - 270;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (1.0 - 2*(float)width_value/(float)texture_width);
					direction[1] = (2*(float)height_value/(float)texture_height - 1.0);
					direction[2] = 1.0f;
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels4[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else
				{
					if(azimuthal_angle>=315.0)
					{
						phi = azimuthal_angle - 360;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}
					else if(azimuthal_angle>=0.0 && azimuthal_angle<45.0)
					{
						phi= azimuthal_angle;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = 1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (2*(float)width_value/(float)texture_width-1.0f);
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels2[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;


				}
				
				
				if(distance<(zfar-0.0001f))
				{
				point[0] = CameraPosition[0] + distance*direction[0];
				point[1] = CameraPosition[1] + distance*direction[1];
				point[2] = CameraPosition[2] + distance*direction[2];
				}
				else
				{
					point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;
				}
				
				
				if(index>=100000)
				{
					index=0;
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];

				}
				else{
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];
				}
				index++;		
				

				if(udpPointCounter<100)
				{
					dStream<<point[0];
					dStream<<point[1];
					dStream<<point[2];	
					udpPointCounter++;
				}
				else
				{
					sendData(UdpSensorData);
					UdpSensorData.clear();
					dStream.device()->reset();
					udpPointCounter = 0;
				}

			}
			if(scanLineIndex>=scanLines-1)	scanLineAdd = -1 ;
			else if(scanLineIndex<=0) scanLineAdd = 1;
			scanLineIndex+=scanLineAdd;

			if(udpPointCounter!=0)
			{
				sendData(UdpSensorData);
				UdpSensorData.clear();
				dStream.device()->reset();
				udpPointCounter = 0;
			}
		//}

		delete [] pixels;
		delete [] pixels2;
		delete [] pixels3;
		delete [] pixels4;

	}
	glBindTexture(GL_TEXTURE_2D, 0);
	//cout<<"numBytes : "<<numBytes<<endl;
	//cout<<"getPointCloud executed"<<endl;

}




void pcl::simulation::LaserSensor::generateRE0xPointCloudRegionScan(GLuint depth_texture_1[],
							int texture_width,
							int texture_height,
							float CameraPosition[],
							float *points,
							float upperBound,
							float lowerBound,
							float angularRight,
							float angularLeft
							)
{
	//Get all the Textures 
	// Texture 1: -45 to 45 degree
	// Texture 2: 45 to 135 degree
	// Texture 3: 135 to 225 degree
	// Texture 4: 225 to 315 degree
	GLint t_width1, t_height1, internalFormat1;
	glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPONENTS, &internalFormat1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &t_width1);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &t_height1);
	
	cout<<"GL_DEPTH_COMPONENT : "<<internalFormat1<<endl;
	GLuint numBytes=0;
	switch(internalFormat1)
	{
		case GL_RGB:
			numBytes = t_width1*t_height1*3;
			break;
		case GL_RGBA:
			numBytes = t_width1*t_height1*4;
			break;
		case GL_DEPTH_COMPONENT:
			numBytes = t_width1*t_height1;
			break;
		default:
			break;
	}

	if(numBytes)
	{
		unsigned char *pixels = new unsigned char[numBytes];
		unsigned char *pixels2 = new unsigned char[numBytes];
		unsigned char *pixels3 = new unsigned char[numBytes];
		unsigned char *pixels4 = new unsigned char[numBytes];

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[0]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[1]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels2);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[2]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels3);

		glBindTexture(GL_TEXTURE_2D, depth_texture_1[3]);
		glGetTexImage(GL_TEXTURE_2D, 0, internalFormat1, GL_UNSIGNED_BYTE, pixels4);


		//local Variables
		 
		int points_per_scanline = samplingFrequency/azimuthal_frequency;
		float scan_start_elevation, scan_end_elevation;
		float elevation_range_per_scanline = (upperBound - lowerBound)/(float)scanLines;
		float distance = 10.0;
		float angular_offset= 0.0, elevation_offset = 0.0, elevation_angle = 0.0, azimuthal_angle = angularRight, height_value, width_value;
		float thetha = 0.0, cos_thetha = 0.0, sin_thetha = 0.0, phi = 0.0, cos_phi =0.0, sin_phi =0.0;
		float direction[3], magnitude, point[3];

		
		// To copy the data into a Byte Array for transfer over UDP
		QByteArray UdpSensorData;
	    QDataStream dStream(&UdpSensorData, QIODevice::WriteOnly);
		int udpPointCounter = 0;
		
		//for(int i=0; i<azimuthal_frequency; i++)
		//{
			scan_start_elevation = upperBound - (elevation_range_per_scanline * scanLineIndex);		//scan_elevation = from top the starting elevation for spiral scan
			scan_end_elevation = 45.0 - (elevation_range_per_scanline * (scanLineIndex+1));

			for(int j=0; j<points_per_scanline; j++)
			{
				int jIndex = (j<(points_per_scanline/2)) ? j : (points_per_scanline - j);
				angular_offset = 2*(angularLeft-angularRight)*((float)jIndex/(float)points_per_scanline);
				elevation_offset = elevation_range_per_scanline*((float)j/(float)points_per_scanline);

				elevation_angle = scan_start_elevation - elevation_offset;	//moving from 45 to -45
				azimuthal_angle = angularRight + angular_offset;
				
				thetha = 90.0 - elevation_angle;
				cos_thetha = cos(thetha*PI/180.0);
				sin_thetha = sin(thetha*PI/180.0);
				
				if(azimuthal_angle<0) azimuthal_angle = 360 + azimuthal_angle;

				phi = azimuthal_angle;
				cos_phi = cos(phi*PI/180.0);
				sin_phi = sin(phi*PI/180.0);
				direction[3];		//direction b/w pixel pos and camera pos
				height_value, width_value ;

				if(azimuthal_angle>=45.0 && azimuthal_angle<135.0)				//texture 1
				{
					phi =  azimuthal_angle - 90;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (2*(float)width_value/(float)texture_width - 1.0);
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = -1.0f;

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}

				else if (azimuthal_angle>=135.0 && azimuthal_angle<225.0)		//texture 3
				{
					phi = azimuthal_angle - 180;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);
					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);

					direction[0] = -1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (1.0f - 2*(float)width_value/(float)texture_width);

					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels3[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else if (azimuthal_angle>=225.0 && azimuthal_angle<315.0)		//texture 4
				{
					phi = azimuthal_angle - 270;
					cos_phi = cos(phi*PI/180.0);
					sin_phi = sin(phi*PI/180.0);

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = (1.0 - 2*(float)width_value/(float)texture_width);
					direction[1] = (2*(float)height_value/(float)texture_height - 1.0);
					direction[2] = 1.0f;
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels4[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;

				}
				else
				{
					if(azimuthal_angle>=315.0)
					{
						phi = azimuthal_angle - 360;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}
					else if(azimuthal_angle>=0.0 && azimuthal_angle<45.0)
					{
						phi= azimuthal_angle;
						cos_phi = cos(phi*PI/180.0);
						sin_phi = sin(phi*PI/180.0);
					}

					height_value = (texture_height/2) + 0.707*(texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
					width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
					direction[0] = 1.0f;
					direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
					direction[2] = (2*(float)width_value/(float)texture_width-1.0f);
					
					magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

					direction[0] /= magnitude;
					direction[1] /= magnitude;
					direction[2] /= magnitude;

					distance = (float)pixels2[3*(int)height_value*t_width1 + 3*(int)width_value]*zfar/ 255.0;


				}
				
				
				if(distance<(zfar-0.0001f))
				{
				point[0] = CameraPosition[0] + distance*direction[0];
				point[1] = CameraPosition[1] + distance*direction[1];
				point[2] = CameraPosition[2] + distance*direction[2];
				}
				else
				{
					point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;
				}
				
				
				if(index>=100000)
				{
					index=0;
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];

				}
				else{
					points[3*index] = point[0];
					points[3*index + 1] = point[1];
					points[3*index + 2] = point[2];
				}
				index++;		


				if(udpPointCounter<100)
				{
					dStream<<point[0];
					dStream<<point[1];
					dStream<<point[2];	
					udpPointCounter++;
				}
				else
				{
					sendData(UdpSensorData);
					UdpSensorData.clear();
					dStream.device()->reset();
					udpPointCounter = 0;
				}

				
			}
			if(scanLineIndex>=scanLines-1)	scanLineAdd = -1 ;
			else if(scanLineIndex<=0) scanLineAdd = 1;
			scanLineIndex+=scanLineAdd;

			if(udpPointCounter!=0)
			{
				sendData(UdpSensorData);
				UdpSensorData.clear();
				dStream.device()->reset();
				udpPointCounter = 0;
			}
		//}

		delete [] pixels;
		delete [] pixels2;
		delete [] pixels3;
		delete [] pixels4;

	}
	glBindTexture(GL_TEXTURE_2D, 0);
	//cout<<"numBytes : "<<numBytes<<endl;
	//cout<<"getPointCloud executed"<<endl;

}



void pcl::simulation::LaserSensor::initialize_()
{
	socket_ = new QUdpSocket(this);

    //Bind Socket to an Address
    /*if(!socket_->bind(QHostAddress::Any, 1235))
    {
        qDebug()<<"Unable to connect to Server";
    }

	QObject::connect(socket_, SIGNAL(readyRead()),
                     this, SLOT(readPendingDatagrams()));
	  */
}


void pcl::simulation::LaserSensor::sendData(QByteArray data)
{	
	QString dataString = "";

	dataString.append(data);

	//qDebug()<<dataString;

	int bytesWritten = socket_->writeDatagram(data, LOCALHOST_IP, LOCALHHOST_PORT);
	int size = data.size();
	data.clear();
}


