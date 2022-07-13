set(MY_LIB_NAME gui_simulation)

message("-----------------------------------------------------------------------------------------")
message("\t[LIB] Neuvisys - ${MY_LIB_NAME}")
message("-----------------------------------------------------------------------------------------")

set(TARGET_NAME "gui_simulation")

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)

set(CATKIN_ENABLE_TESTING OFF)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package(libcaer REQUIRED)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(SRC_BASE_PATH "src")
set(HDRS_BASE_PATH "include/${MY_LIB_NAME}")

set(NEUVISYS_GUI_SIMU_SOURCES        
        ${SRC_BASE_PATH}/NeuvisysguiSimulation.cpp
        ${SRC_BASE_PATH}/NeuvisysthreadSimulation.cpp
        ${SRC_BASE_PATH}/GLWidget.cpp
		${SRC_BASE_PATH}/neuvisysguisimulation.ui
)

set(NEUVISYS_GUI_SIMU_HEADERS       
		${HDRS_BASE_PATH}/GLWidget.h
        ${HDRS_BASE_PATH}/NeuvisysguiSimulation.h
        ${HDRS_BASE_PATH}/NeuvisysthreadSimulation.h
)

set(NEUVISYS_GUI_SIMU_LIBS
	Qt${QT_VERSION_MAJOR}::Widgets 
	Qt${QT_VERSION_MAJOR}::Charts
	libcaer::caer
	${catkin_LIBRARIES}
)

set(NEUVISYS_GUI_SIMU_INCLUDE_DIRS
	${libcaer_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_library(${MY_LIB_NAME} STATIC ${NEUVISYS_GUI_SIMU_SOURCES} ${NEUVISYS_GUI_SIMU_HEADERS})

target_include_directories(
	${MY_LIB_NAME} 
	PUBLIC include/
	PROTECTED ${NEUVISYS_GUI_SIMU_INCLUDE_DIRS}
	PRIVATE ${HDRS_BASE_PATH}
)
target_link_libraries(
	${MY_LIB_NAME} 
	PUBLIC ${NEUVISYS_GUI_SIMU_LIBS} 
	PRIVATE motor_control network camera simulator
)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
