#include <iostream>
#include <fstream>
#include <cstring>
#include <QDebug>

#include <pcl/simulation/laser_sensor.h>

using namespace std;


GLuint VBO;
GLuint ProgramId, VertexShaderId, FragmentShaderId;


pcl::simulation::LaserSensor::LaserSensor()
{
		//Initialization of Variables for an instance of the
		//laser sensor class. These variables will determine
		//the scanning behavior(not scan pattern but other properties
		//range, accuracy etc.)
	

		//Default Sampling and Azimuthal Frequency
		_index=0;
		_samplingFrequency = 5000;
		_azimuthal_frequency = 5;
		
		//Near Plane Far Plane of the Camera
		_znear = 1.0f;
		_zfar = 30.0f;
		
		//Default Number of Scanlines
		_scanLineIndex = 32;
		_scanLines = 64;
		_scanLineAdd=1;
		
		//Position of the Laser Sensor 
		_CameraPosition[0] = 0.0f;
		_CameraPosition[1] = 2.0f;
		_CameraPosition[2] = 0.0f;

		//Orientaion of the Camera
		_CameraOrientation[0] = 0.0f;
		_CameraOrientation[1] = 1.0f;
		_CameraOrientation[2] = 0.0f;
}


pcl::simulation::LaserSensor::~LaserSensor()
{

}

/*
*	This method renders the scene to the textures. Based on the camera location
*	and orientation, the scene is rendered to the texture (using OpenGL technique
*	'render to Texture'). Here we only require the depth information of the
*	scene, so we take the depth component of the scene and save it to the corresponding
*	depth texture. Here the Depth is calculated from the image plane and perpendicular 
*	to it. So we have yet to calculate the actual depth from the Sensor.
*/
void  pcl::simulation::LaserSensor::renderSceneToDepthTexture(GLuint &dfbo, 
																float lookAt[],
																float upDirection[],
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
	glFrustum(-1.0f, 1.0f, -1.0f, 1.0f, _znear, _zfar);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(_CameraPosition[0], _CameraPosition[1], _CameraPosition[2],
			lookAt[0], lookAt[1], lookAt[2],
			upDirection[0],upDirection[1], upDirection[2]);
	scene_->draw();
	
	glBindFramebuffer (GL_FRAMEBUFFER, 0);
    //-----------------------Initial Depth texture End------------------------------
	
	glViewport(0,0, (float)window_width, (float)window_height);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
}



/*
 *	Here the input provide to the method is a depth texture of the scene. The depth 
 *	texture (generated by RenderScenetoDepthTexture()) has relative depth values w.r.t
 *	the rendering image plane. This function converts the relative depth values to Real
 *	depth values by adding an offset value(differs from pixel to pixel). This is done 
 *	using GL shaders where each pixel of the depth image is processed in a seperate 
 *	thread over GPU. 
 */
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
		GLuint textureLength = glGetUniformLocation(ProgramId, "textureLength");
		glUniform1f(location_znear, _znear);
		glUniform1f(location_zfar, _zfar);
		glUniform1f(textureLength, texture_height);

		GLenum err = glGetError();

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



bool pcl::simulation::LaserSensor::generateRenderingDepthTextures(GLuint &dtexture, 
																	GLuint &dfbo,
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
	GLenum err = glGetError();
	
	glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, texture_width, texture_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	
	err = glGetError();
	if(err == GL_OUT_OF_MEMORY)
	{
		std::cout<<"GL_OUT_OF_MEMORY Error"<<std::endl;
	}
	else if(err == GL_INVALID_OPERATION)
	{
		std::cout<<"GL_INVALID_OPERATION Error"<<std::endl;
	}
	else if(err ==  GL_INVALID_VALUE)
	{
		std::cout<<"GL_INVALID_VALUE Error"<<std::endl;		
	}
	else if(err == GL_INVALID_ENUM)
	{
		std::cout<<"GL_INVALID_ENUM  Error"<<std::endl;		
	}
	else
	{
		std::cout<<"Unknown Error : "<<err<<std::endl;
	}
	glBindTexture (GL_TEXTURE_2D, 0);

	//generate framebuffer to attach to the depth texture
	glGenFramebuffers (1, &dfbo);
	glBindFramebuffer (GL_FRAMEBUFFER, dfbo);

	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	glFramebufferTexture2D (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dtexture, 0);
	
	
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if(status!= GL_FRAMEBUFFER_COMPLETE)
	{
		if(status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT"<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT"<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_UNSUPPORTED)
		{
			std::cout<<"GL_FRAMEBUFFER_UNSUPPORTED"<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_UNDEFINED )
		{
			std::cout<<"GL_FRAMEBUFFER_UNDEFINED "<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER )
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER "<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER )
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER "<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE  )
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE  "<<std::endl;
		}
		else if(status == GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS   )
		{
			std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS  "<<std::endl;
		}
		
			std::cout<<"Error In Creating RenderBuffer"<<std::endl;
			return false;
	}
	glBindFramebuffer (GL_FRAMEBUFFER, 0);

	std::cout<<"reached here renderbuffer"<<std::endl;
	return true;
}


bool pcl::simulation::LaserSensor::generateOffsetTextures(GLuint &dtexture_offset, 
															GLuint &dfbo_offset,
															int texture_width,
															int texture_height)
{
	

	//create offset textures
	glGenTextures(1, &dtexture_offset);
	glBindTexture(GL_TEXTURE_2D, dtexture_offset);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	GLenum err = glGetError();
	glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, texture_width, texture_height, 0, GL_RGB, GL_FLOAT, NULL); // add offset data to offset buffer
	err = glGetError();
	if(err == GL_OUT_OF_MEMORY)
	{
		std::cout<<"GL_OUT_OF_MEMORY Error"<<std::endl;
	}
	else if(err == GL_INVALID_OPERATION)
	{
		std::cout<<"GL_INVALID_OPERATION Error"<<std::endl;
	}
	else if(err ==  GL_INVALID_VALUE)
	{
		std::cout<<"GL_INVALID_VALUE Error"<<std::endl;		
	}
	else if(err == GL_INVALID_ENUM)
	{
		std::cout<<"GL_INVALID_ENUM  Error"<<std::endl;		
	}
	else
	{
		std::cout<<"Unknown Error : "<<err<<std::endl;
	}
	glBindTexture (GL_TEXTURE_2D, 0);

	GLint maxSize;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxSize);
	std::cout<<"GL_MAX_TEXTURE_SIZE : "<< maxSize<<std::endl;
	

	//generate framebuffer to attach to the depth texture
	glGenFramebuffers (1, &dfbo_offset);
	glBindFramebuffer (GL_FRAMEBUFFER, dfbo_offset);
	glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dtexture_offset, 0);
	
	GLenum status_1 = glCheckFramebufferStatus(GL_FRAMEBUFFER);

	if(status_1!= GL_FRAMEBUFFER_COMPLETE) 
	{
		std::cout<<"Error In Creating OffsetBuffer"<<std::endl;
		return false;
	}
	glBindFramebuffer (GL_FRAMEBUFFER, 0);


	std::cout<<"reached here"<<std::endl;
	return true;
}


/*
This shader is for displaying the Scene(colour scene for reference).
*/
bool pcl::simulation::LaserSensor::CreateShaders()
{
    GLenum ErrorCheckValue = glGetError();
    
	
	//Vertex Shader Initialization
    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	ifstream ifs("shader.vert");
	std::string str((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
	GLchar *cstr = new GLchar[str.size() + 1];
	strcpy(cstr, str.c_str());
	const GLchar *vertexShader = cstr;
	glShaderSource(VertexShaderId, 1, &vertexShader, NULL);
    glCompileShader(VertexShaderId);


	//Fragment Shader Initialization
    FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
    ifstream ifsf("shader.frag");
	std::string strf((std::istreambuf_iterator<char>(ifsf)),(std::istreambuf_iterator<char>()));
	GLchar *cstrf = new GLchar[strf.size() + 1];
	strcpy(cstrf, strf.c_str());
	const GLchar *fragmentShader = cstrf;
	glShaderSource(FragmentShaderId, 1, &fragmentShader, NULL);
    glCompileShader(FragmentShaderId);

	
	//Creating Final Program by Attaching both the shaders
    ProgramId = glCreateProgram();
        glAttachShader(ProgramId, VertexShaderId);
        glAttachShader(ProgramId, FragmentShaderId);
    glLinkProgram(ProgramId);

	

	GLint status;
    glGetProgramiv (ProgramId, GL_LINK_STATUS, &status);

	//If there is an error in creating Shader Program
    if (status == GL_FALSE)
    {
		GLint maxLength = 0;
		glGetProgramiv(ProgramId, GL_INFO_LOG_LENGTH, &maxLength);

		//The maxLength includes the NULL character
		std::vector<GLchar> infoLog(maxLength);
		glGetProgramInfoLog(ProgramId, maxLength, &maxLength, &infoLog[0]);

		std::cout<<"Max Length : "<<maxLength<<std::endl;

		for (std::vector<char>::const_iterator i = infoLog.begin(); i != infoLog.end(); ++i)
			std::cout << *i << ' ';

		for(int j=0; j<infoLog.size(); j++)
		{
			std::cout<< infoLog[j] <<' ';
		}

		//The program is useless now. So delete it.
		glDeleteProgram(ProgramId);

		//Provide the infolog in whatever manner you deem best.
		//Exit with failure.
		return false;
    }


    ErrorCheckValue = glGetError();
    if (ErrorCheckValue != GL_NO_ERROR)
    {								
        fprintf(
            stderr,
            "ERROR: Could not create the shaders: %s \n",
            gluErrorString(ErrorCheckValue)
        );

		return false; 
    }

	return true;
}



void pcl::simulation::LaserSensor::generateRE0xPointCloudFullScan(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points		
							)
{
	_azimuthal_frequency = Azimuthal_Freq;
	if(_scanLines != NumScanlines)
	{
		_scanLines = NumScanlines;
		_scanLineIndex = 0;
	}
	
	performScan(ScanPatternType::FULL_FIELD_SCAN, final_depth_texture_fbo, texture_width, texture_height, points, 35, -35, 0, 360);

}




void pcl::simulation::LaserSensor::generateRE0xPointCloudBoundedElevation(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points,
							float upperBound,
							float lowerBound
							)
{
	_azimuthal_frequency = Azimuthal_Freq;
	if(_scanLines != NumScanlines)
	{
		_scanLines = NumScanlines;
		_scanLineIndex = 0;
	}

	performScan(ScanPatternType::BOUNDED_ELEVATION_SCAN, final_depth_texture_fbo, texture_width, texture_height,  points, upperBound, lowerBound, 0, 360);

}




void pcl::simulation::LaserSensor::generateRE0xPointCloudRegionScan(GLuint final_depth_texture_fbo[],
							int Azimuthal_Freq,
							int NumScanlines,
							int texture_width,
							int texture_height,
							float *points,
							float upperBound,
							float lowerBound,
							float angularRight,
							float angularLeft
							)
{
	_azimuthal_frequency = Azimuthal_Freq;
	if(_scanLines != NumScanlines)
	{
		_scanLines = NumScanlines;
		_scanLineIndex = 0;
	}

	performScan(ScanPatternType::REGION_SCAN, final_depth_texture_fbo, texture_width, texture_height, points, upperBound, lowerBound, angularRight, angularLeft);

}




void pcl::simulation::LaserSensor::performScan(	ScanPatternType scanMode,
												GLuint final_depth_texture_fbo[],
												int texture_width, 
												int texture_height,
												float *points,
												float upperBound,
												float lowerBound,
												float angularRight,
												float angularLeft)
{

	//Get all the Textures 
	// Texture 1: -45 to 45 degree
	// Texture 2: 45 to 135 degree
	// Texture 3: 135 to 225 degree
	// Texture 4: 225 to 315 degree
		
	//local Variables
	int points_per_scanline = _samplingFrequency/_azimuthal_frequency;

	//Starting and Ending Elevation of the current Scanline 
	float scan_start_elevation, scan_end_elevation;

	//Range of the Current Scanline
	float elevation_range_per_scanline = (upperBound - lowerBound)/(float)_scanLines;

	//Maximum distance - farthest Point
	float distance = 10.0;

		
		
	float elevation_angle = 0.0;
	float elevation_offset = 0.0;
		
	float azimuthal_angle = 0.0;
	float azimuthal_offset= 0.0;

	//Height Value and Width value of a particular point in a texture
	float height_value;
	float width_value;


	//Representing any sensed point in Spherical Coordinate System i.e. currently 
	//intialization of variables related to Spherical Coordinate System
	float thetha = 0.0, cos_thetha = 0.0, sin_thetha = 0.0;
	float phi = 0.0, cos_phi =0.0, sin_phi =0.0;


		
	// To copy the data into a Byte Array for transfer over UDP
	QByteArray UdpSensorData;
	QDataStream dStream(&UdpSensorData, QIODevice::WriteOnly);
	int udpPointCounter = 0;


	char* udpData = new char[900];
		
	
		
	if(_scanLineAdd==1)
	{
		scan_start_elevation = upperBound - (elevation_range_per_scanline * _scanLineIndex);		//scan_elevation = from top the starting elevation for spiral scan
	}
	else
	{
		scan_start_elevation = lowerBound + (elevation_range_per_scanline * (_scanLines-1-_scanLineIndex));
	}

	//To consider the case when we have to scan the region between 340 degree(angularRight) to 20 degree(angularLeft)
	if(angularLeft<angularRight) angularLeft += 360;

	for(int j=0; j<points_per_scanline; j++)
	{

			
		//Direction and location of the current Point
		float direction[3], point[3];

		//Since movement of the sensor is different azimuthally for Region Scam
		if(scanMode == ScanPatternType::REGION_SCAN)
		{
			int jIndex = (j<(points_per_scanline/2)) ? j : (points_per_scanline - j);
			azimuthal_offset = 2*(angularLeft-angularRight)*((float)jIndex/(float)points_per_scanline);
		}
		else
		{
			azimuthal_offset = 360.0*((float)j/(float)points_per_scanline);
		}


		//Calculating Azimuthal Angle of the Current Point
		azimuthal_angle = angularRight + azimuthal_offset;
		if(azimuthal_angle>360) 
		{
			azimuthal_angle = azimuthal_angle - 360;				//check if this condition is ever true
		}
		else if (azimuthal_angle<0)
		{
			azimuthal_angle = azimuthal_angle + 360;
		}


		//Calculating Elevation Angle of the Current Point
		elevation_offset = elevation_range_per_scanline*((float)j/(float)points_per_scanline);
		elevation_angle = scan_start_elevation - (_scanLineAdd*elevation_offset);	//moving from 45 to -45
			
				
		//Calculating Polar Angle and Azimuth Angle in Spherical Coordinate System
		thetha = 90.0 - elevation_angle;
		cos_thetha = cos(thetha*PI/180.0);
		sin_thetha = sin(thetha*PI/180.0);
		phi = azimuthal_angle;
		cos_phi = cos(phi*PI/180.0);
		sin_phi = sin(phi*PI/180.0);
			
		if(azimuthal_angle>45.0 && azimuthal_angle<=135.0)				//texture 1
		{
			phi =  azimuthal_angle - 90;
			cos_phi = cos(phi*PI/180.0);
			sin_phi = sin(phi*PI/180.0);

			height_value = (texture_height/2) + (texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
			width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
			direction[0] = (2*(float)width_value/(float)texture_width - 1.0);
			direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
			direction[2] = -1.0f;

			float magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

			direction[0] /= magnitude;
			direction[1] /= magnitude;
			direction[2] /= magnitude;

			glBindFramebuffer (GL_FRAMEBUFFER, final_depth_texture_fbo[0]);

			GLubyte rgb[4];
			glReadPixels((int)width_value, (int)height_value, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgb);
			
			float tempDistance = rgb[0] + rgb[1] * 256.0 + rgb[2] * 256.0 * 256.0;;
			tempDistance/=16777216;
			
			distance = tempDistance*_zfar;

			glBindFramebuffer (GL_FRAMEBUFFER, 0);

		}

		else if (azimuthal_angle>135.0 && azimuthal_angle<=225.0)		//texture 3
		{
			phi = azimuthal_angle - 180;
			cos_phi = cos(phi*PI/180.0);
			sin_phi = sin(phi*PI/180.0);
			height_value = (texture_height/2) + (texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
			width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);

			direction[0] = -1.0f;
			direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
			direction[2] = (1.0f - 2*(float)width_value/(float)texture_width);

			float magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

			direction[0] /= magnitude;
			direction[1] /= magnitude;
			direction[2] /= magnitude;
		
			glBindFramebuffer (GL_FRAMEBUFFER, final_depth_texture_fbo[2]);

			GLubyte rgb[4];
			glReadPixels((int)width_value, (int)height_value, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgb);
			
			float tempDistance = rgb[0] + rgb[1] * 256.0 + rgb[2] * 256.0 * 256.0;;
			tempDistance/=16777216;
			
			distance = tempDistance*_zfar;

			glBindFramebuffer (GL_FRAMEBUFFER, 0);

		}
		else if (azimuthal_angle>225.0 && azimuthal_angle<=315.0)		//texture 4
		{
			phi = azimuthal_angle - 270;
			cos_phi = cos(phi*PI/180.0);
			sin_phi = sin(phi*PI/180.0);

			height_value = (texture_height/2) + (texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
			width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
			direction[0] = (1.0 - 2*(float)width_value/(float)texture_width);
			direction[1] = (2*(float)height_value/(float)texture_height - 1.0);
			direction[2] = 1.0f;
					
			float magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);

			direction[0] /= magnitude;
			direction[1] /= magnitude;
			direction[2] /= magnitude;

			glBindFramebuffer (GL_FRAMEBUFFER, final_depth_texture_fbo[3]);

			GLubyte rgb[4];
			glReadPixels((int)width_value, (int)height_value, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgb);
			
			float tempDistance = rgb[0] + rgb[1] * 256.0 + rgb[2] * 256.0 * 256.0;;
			tempDistance/=16777216;
			
			distance = tempDistance*_zfar;

			glBindFramebuffer (GL_FRAMEBUFFER, 0);

		}
		else
		{
			if(azimuthal_angle>315.0)
			{
				phi = azimuthal_angle - 360;
				cos_phi = cos(phi*PI/180.0);
				sin_phi = sin(phi*PI/180.0);
			}
			else if(azimuthal_angle>=0.0 && azimuthal_angle<=45.0)
			{
				phi= azimuthal_angle;
				cos_phi = cos(phi*PI/180.0);
				sin_phi = sin(phi*PI/180.0);
			}

			height_value = (texture_height/2) + (texture_height/2)*(cos_thetha/sin_thetha)*(1/cos_phi);
			width_value = (texture_width/2) - (texture_width/2)*(sin_phi/cos_phi);
					
			direction[0] = 1.0f;
			direction[1] = (2*(float)height_value/(float)texture_height  - 1.0);
			direction[2] = (2*(float)width_value/(float)texture_width-1.0f);
					
			float magnitude = sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);


			//Normalizing the direction
			direction[0] /= magnitude;
			direction[1] /= magnitude;
			direction[2] /= magnitude;
				
			glBindFramebuffer (GL_FRAMEBUFFER, final_depth_texture_fbo[1]);

			GLubyte rgb[4];
			glReadPixels((int)width_value, (int)height_value, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgb);
			
			float tempDistance = rgb[0] + rgb[1] * 256.0 + rgb[2] * 256.0 * 256.0;;
			tempDistance/=16777216;
			
			distance = tempDistance*_zfar;

			glBindFramebuffer (GL_FRAMEBUFFER, 0);


		}
				
				
		if(distance<(_zfar-0.0001f))
		{
			float rotatedDir[3];
			multiplyRotationalMatrix(&direction[0], &rotatedDir[0]);
			point[0] = _CameraPosition[0] + distance*rotatedDir[0];
			point[1] = _CameraPosition[1] + distance*rotatedDir[1];
			point[2] = _CameraPosition[2] + distance*rotatedDir[2];
		}
		else
		{
			point[0] = 0.0; point[1] = 0.0; point[2] = 0.0;
			distance = 0.0;
		}

		if(j==47)
		{
			//std::cout<<"Point Value x : "<<point[0]<<" y : "<<point[1]<<" z : "<<point[2]<<std::endl; 
		}
				
		if(_index>=100000)
		{
			_index=0;
			points[3*_index] = point[0];
			points[3*_index + 1] = point[1];
			points[3*_index + 2] = point[2];

		}
		else{
			points[3*_index] = point[0];
			points[3*_index + 1] = point[1];
			points[3*_index + 2] = point[2];
		}
		_index++;		


		if(udpPointCounter<100)
		{
			unsigned short azimuthAngle = (unsigned short)(azimuthal_angle*100);
			short elevationAngle = (short)(elevation_angle*100);
			unsigned short range = (unsigned short)(distance*1000);
			char reserved = '-';
			char intensity = '0';

				
			unsigned char pointData[8];
			pointData[0] = 	azimuthAngle & 0xFF;
			pointData[1] = 	(azimuthAngle>>8) & 0xFF;
			pointData[2] = 	elevationAngle & 0xFF;
			pointData[3] = 	(elevationAngle>>8) & 0xFF;
			pointData[4] = 	range & 0xFF;
			pointData[5] = 	(range>>8) & 0xFF;
			pointData[6] = 	reserved;
			pointData[7] = 	intensity;
				


			const char* pointDataChar = reinterpret_cast<const char*>(pointData);
				
			memcpy(&udpData[8*udpPointCounter], pointDataChar, 8);
			udpPointCounter++;

		}
		else
		{
			sendData(udpData, 800);
			UdpSensorData.clear();
			dStream.device()->reset();
				

			udpPointCounter = 0;
		}

				
	}
	if(_scanLineIndex>=_scanLines-1)	_scanLineAdd = -1 ;
	else if(_scanLineIndex<=0) _scanLineAdd = 1;
	_scanLineIndex+=_scanLineAdd;

	if(udpPointCounter!=0)
	{
		//sendData(UdpSensorData); //emit signal
		sendData(udpData, 800);
		UdpSensorData.clear();
		dStream.device()->reset();
		udpPointCounter = 0;
	}
}


void pcl::simulation::LaserSensor::startClock()
{
	//Start the Sensor Clock 60MHz
}



void pcl::simulation::LaserSensor::multiplyRotationalMatrix(float *inVec, float *outVec)
{
	float rotMatrix[3][3];

	float c3 =  cos(_CameraOrientation[0]); //Roll
	float c2 =  cos(_CameraOrientation[1]); //Pitch
	float c1 =  cos(_CameraOrientation[2]); //Yaw

	float s3 =  sin(_CameraOrientation[0]); //Roll
	float s2 =  sin(_CameraOrientation[1]); //Pitch
	float s1 =  sin(_CameraOrientation[2]); //Yaw

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