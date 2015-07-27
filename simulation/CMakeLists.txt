set(SUBSYS_NAME simulation)
set(SUBSYS_DESC "Point Cloud Library Simulation")
set(SUBSYS_DEPS common io surface kdtree features search octree visualization filters geometry)


set(build TRUE)
find_package(OpenGL)
find_package(GLEW)

find_package(Qt 5.3.2 REQUIRED QtCore QtOpenGL QtGui)
find_package(Qt5Core)

SET(CMAKE_AUTOMOC ON) 

SET(QT_USE_QTNETWORK TRUE)


###############################
#find_package(GLUT_REQUIRED)
#include_directories(${GLUT_INCLUDE_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
#if(NOT GLUT_FOUND)
#	message(ERROR "GLUT not found")
#endif(NOT GLUT_FOUND)
###############################


PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS opengl glew)

PCL_ADD_DOC("${SUBSYS_NAME}")
 
if(build)
    set(srcs 
         src/camera.cpp
         src/model.cpp
         src/range_likelihood.cpp
         src/scene.cpp
         src/glsl_shader.cpp
         src/sum_reduce.cpp
         src/laser_sensor.cpp
		 src/laser_sensor_wrapper.cpp
        )

    set(incs 
        "include/pcl/${SUBSYS_NAME}/camera.h"
        "include/pcl/${SUBSYS_NAME}/model.h"
        "include/pcl/${SUBSYS_NAME}/range_likelihood.h"
        "include/pcl/${SUBSYS_NAME}/scene.h"
        "include/pcl/${SUBSYS_NAME}/glsl_shader.h"
        "include/pcl/${SUBSYS_NAME}/sum_reduce.h"
        "include/pcl/${SUBSYS_NAME}/laser_sensor.h"
		"include/pcl/${SUBSYS_NAME}/laser_sensor_wrapper.h"
        )


	

	
	
	#set(LASER_SENSOR_HEADER )
	#QT4_WRAP_CPP(LASER_SENSOR_HEADERS_MOC ${LASER_SENSOR_HEADER})
	
		
    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include"
                        "${GLEW_INCLUDE_DIR}")
	include_directories(${VTK_INCLUDE_DIRS})
	include_directories(${QT_INCLUDE_DIR})

	# Find the QtWidgets library
	find_package(Qt5Network)

	# Add the include directories for the Qt 5 Widgets module to
	# the compile lines.
	#include_directories(${Qt5Network_INCLUDE_DIRS})

	# Use the compile definitions defined in the Qt 5 Widgets module
	#add_definitions(${Qt5Network_DEFINITIONS})

	# Add compiler flags for building executables (-fPIE)
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Network_EXECUTABLE_COMPILE_FLAGS}")

	
	#qt5_generate_moc(include/pcl/${SUBSYS_NAME}/laser_sensor.h laser_sensor.moc)
						
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} 
												   ${incs} )

												   
    target_link_libraries("${LIB_NAME}" pcl_common 
										pcl_io
										${OPENGL_LIBRARIES} 
										${GLEW_LIBRARIES} 
										${QT_LIBRARIES} 
										Qt5::Core 
										Qt5::Network
										${VTK_IO_TARGET_LINK_LIBRARIES})
			  

						  
    set(EXT_DEPS eigen3)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
                      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    #PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

    add_subdirectory(tools)


	
endif(build)
