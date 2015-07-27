find_package(GLEW)

find_package(Qt  5.3.2 REQUIRED QtCore QtOpenGL QtNetwork QtGui)
find_package(Qt5Network)

SET(QT_USE_QTNETWORK TRUE)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

if(GLEW_FOUND)
  find_package(GLUT)
  if(GLUT_FOUND)
    include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${QT_INCLUDE_DIR})
    include_directories(${VTK_INCLUDE_DIRS})
	include_directories(${QT_INCLUDE_DIR})
	include_directories(${CMAKE_CURRENT_SOURCE_DIR})

	INCLUDE(${QT_USE_FILE})
	ADD_DEFINITIONS(${QT_DEFINITIONS})

	#Adding Executables for pcl_simulation_viewer
    PCL_ADD_EXECUTABLE(pcl_sim_viewer ${SUBSYS_NAME} sim_viewer.cpp)
    target_link_libraries (pcl_sim_viewer	${VTK_IO_TARGET_LINK_LIBRARIES}    
											pcl_kdtree 
											pcl_simulation   
											pcl_common  
											pcl_io 
											pcl_visualization   
											${GLEW_LIBRARIES} 
											${GLUT_LIBRARIES} 
											${OPENGL_LIBRARIES} 
											${GLEW_LIBRARIES})

	#Adding Executables for pcl_sim_test_simple  
    PCL_ADD_EXECUTABLE(pcl_sim_test_simple ${SUBSYS_NAME} sim_test_simple.cpp)
    target_link_libraries (pcl_sim_test_simple	${VTK_IO_TARGET_LINK_LIBRARIES}   
												pcl_simulation   
												pcl_common  
												pcl_io 
												pcl_visualization 
												${GLEW_LIBRARIES} 
												${GLUT_LIBRARIES} 
												${OPENGL_LIBRARIES} 
												${GLEW_LIBRARIES})
	  
	
	
	
	#Adding Executables for pcl_simulation_sensor
	SET(APPLICATION_FORMS applicationwindow.ui)	
	QT4_WRAP_UI(APPLICATION_FORMS_HEADERS ${APPLICATION_FORMS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
	PCL_ADD_EXECUTABLE(pcl_simulation_sensor ${SUBSYS_NAME} simulation_sensor.cpp
															applicationwindow.cpp
															laserglwidget.cpp
															laserglwidget.h
															applicationwindow.h
															lasersensorthread.cpp
															lasersensorthread.h
															${MAINWINDOW_HEADERS_MOC}
															${APPLICATION_FORMS_HEADERS})
    target_link_libraries (pcl_simulation_sensor	${VTK_IO_TARGET_LINK_LIBRARIES}   
													pcl_kdtree
													pcl_simulation   
													pcl_common  
													pcl_io 
													pcl_visualization 
													${GLEW_LIBRARIES} 
													${GLUT_LIBRARIES} 
													${OPENGL_LIBRARIES} 
													${GLEW_LIBRARIES} 
													${QT_LIBRARIES} 
													Qt5::Network )
													
													
	#Adding Executables for pcl_simulation_sensor_console
	SET(CONSOLE_FORMS consoleWindow.ui)	
	QT4_WRAP_UI(CONSOLE_FORMS_HEADERS ${CONSOLE_FORMS})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
	PCL_ADD_EXECUTABLE(pcl_simulation_sensor_console ${SUBSYS_NAME} simulation_sensor_console.cpp
																	consoleWindow.cpp
																	consoleWindow.h
																	${CONSOLE_FORMS_HEADERS})
	target_link_libraries(pcl_simulation_sensor_console	pcl_simulation   
														${QT_LIBRARIES})
	
	
	#Adding Executables for pcl_sim_test_performance
    PCL_ADD_EXECUTABLE(pcl_sim_test_performance ${SUBSYS_NAME} sim_test_performance.cpp)
    target_link_libraries (pcl_sim_test_performance 	${VTK_IO_TARGET_LINK_LIBRARIES} 
														pcl_simulation   
														pcl_common  
														pcl_io 
														pcl_visualization
														${GLEW_LIBRARIES} 
														${GLUT_LIBRARIES} 
														${OPENGL_LIBRARIES} 
														${GLEW_LIBRARIES})

														

	#Adding Library named Simulation_io
    set(srcs simulation_io.cpp )
    set(incs simulation_io.hpp )
    set(LIB_NAME pcl_simulation_io)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME}	${srcs} 
												${incs} 
												${compression_incs} 
												${impl_incs}
												${VTK_IO_TARGET_LINK_LIBRARIES} 
												${OPENNI_INCLUDES})
    target_link_libraries(${LIB_NAME} 	pcl_simulation 
										pcl_common 
										pcl_io 
										${VTK_IO_TARGET_LINK_LIBRARIES} 
										${OPENGL_LIBRARIES} 
										${GLUT_LIBRARIES})

	#Addling Executables for pcl_sim_terminal_demo								
    PCL_ADD_EXECUTABLE(pcl_sim_terminal_demo ${SUBSYS_NAME} sim_terminal_demo.cpp)
    target_link_libraries (pcl_sim_terminal_demo	${VTK_IO_TARGET_LINK_LIBRARIES}    
													pcl_simulation   
													pcl_common  
													pcl_io 
													pcl_visualization 
													pcl_simulation_io
													${GLEW_LIBRARIES} 
													${GLUT_LIBRARIES} 
													${OPENGL_LIBRARIES} 
													${GLEW_LIBRARIES})


		
	
	#Change Later to add this directly from Simulation_library CMakeList
	set (SHADER_FILE_DIRECTORY ${CMAKE_SOURCE_DIR}/simulation/src)
	set (SHADER_BINARY_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	MESSAGE("${SHADER_FILE_DIRECTORY}")
	MESSAGE("${SHADER_BINARY_DIRECTORY}")

	add_custom_command(TARGET pcl_simulation_sensor 
						POST_BUILD 
						COMMAND ${CMAKE_COMMAND} -E copy ${SHADER_FILE_DIRECTORY}/shader.vert ${CMAKE_CURRENT_BINARY_DIR})
						
	add_custom_command(TARGET pcl_simulation_sensor 
						POST_BUILD 
						COMMAND ${CMAKE_COMMAND} -E copy ${SHADER_FILE_DIRECTORY}/shader.frag ${CMAKE_CURRENT_BINARY_DIR})

  endif(GLUT_FOUND)
endif (GLEW_FOUND)
