cmake_minimum_required(VERSION 3.10.0)

project(neuvisys-simulation-qt)

message("-----------------------------------------------------------------------------------------")
message("Neuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

set(TARGET_NAME "neuvisys-simulation-qt")

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package(libcaer REQUIRED)

set(NEUVISY_GUI_SIMU_SOURCES
        neuvisysguisimulation.ui
        GuiSimulation.cpp
        NeuvisysguiSimulation.cpp
        NeuvisysthreadSimulation.cpp
        GLWidget.cpp
)

set(NEUVISYS_GUI_SIMU_LIBS
	Qt${QT_VERSION_MAJOR}::Widgets 
	Qt${QT_VERSION_MAJOR}::Charts
)

set(NEUVISYS_GUI_SIMU_INCLUDE_DIRS
	${catkin_INCLUDE_DIRS}
)

#-------------------------------------------------------------------------------
if (${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-simulation-qt
        ${NEUVISYS_GUI_SIMU_SOURCES}
        ${NEUVISYS_CAMERA_SOURCES}
        ${NEUVISYS_SIMULATOR_SOURCES}
        ${NEUVISYS_NETWORK_SOURCES}
        ${NEUVISYS_MOTOR_SOURCES}
    )
else ()
    if (ANDROID)
        add_library(neuvisys-simulation-qt SHARED 
                ${NEUVISYS_GUI_SIMU_SOURCES}
                ${NEUVISYS_CAMERA_SOURCES}
                ${NEUVISYS_SIMULATOR_SOURCES}
                ${NEUVISYS_NETWORK_SOURCES}
                ${NEUVISYS_MOTOR_SOURCES}
                )
    else ()
        add_executable(neuvisys-simulation-qt GuiSimulation.cpp NeuvisysguiSimulation.cpp NeuvisysthreadSimulation.cpp GLWidget.cpp
                ${NEUVISYS_GUI_SIMU_SOURCES}
                ${NEUVISYS_CAMERA_SOURCES}
                ${NEUVISYS_SIMULATOR_SOURCES}
                ${NEUVISYS_NETWORK_SOURCES}
                ${NEUVISYS_MOTOR_SOURCES}
                )
    endif ()
endif ()


target_link_libraries(neuvisys-simulation-qt 
	PUBLIC 	${NEUVISYS_LIBS} 
			${NEUVISYS_GUI_SIMU_LIBS} 
			${NEUVISYS_NETWORK_LIBS} 
			${NEUVISYS_SIMULATOR_LIBS} 
			${NEUVISYS_CAMERA_LIBS} 
			${NEUVISYS_MOTOR_LIBS}
)
target_include_directories(neuvisys-simulation-qt PUBLIC ${NEUVISYS_INCLUDE_DIRS} ${NEUVISYS_GUI_SIMU_INCLUDE_DIRS})
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(NEUVISY_GUI_SIMU_SOURCES ${NEUVISY_GUI_SIMU_SOURCES} PARENT_SCOPE)
set(NEUVISYS_GUI_SIMU_LIBS ${NEUVISYS_GUI_SIMU_LIBS} PARENT_SCOPE)
set(NEUVISYS_GUI_SIMU_INCLUDE_DIRS ${NEUVISYS_GUI_SIMU_INCLUDE_DIRS} PARENT_SCOPE)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
