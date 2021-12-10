set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

include_directories(${catkin_INCLUDE_DIRS})

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../network/Utils.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp)

set(ROS_SOURCES
        ../robot-control/SimulationInterface.cpp
        ../robot-control/FrameToEvents.cpp
        ../robot-control/Motor.cpp
        ../robot-control/motor-control/BrushlessMotor.cpp
        ../robot-control/motor-control/libserie/Faulhaber.cpp
        ../robot-control/motor-control/libserie/comserie.cpp)

set(GUI_SOURCES
        guiMain.cpp
        Neuvisysgui.cpp
        Neuvisysthread.cpp
        GLWidget.cpp
        neuvisysgui.ui)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-qt
            ${GUI_SOURCES}
            ${PROJECT_SOURCES}
            ${ROS_SOURCES}
            )
else()
    if(ANDROID)
        add_library(neuvisys-qt SHARED
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${ROS_SOURCES}
                )
    else()
        add_executable(neuvisys-qt
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${ROS_SOURCES}
                )
    endif()
endif()

target_link_libraries(neuvisys-qt PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${catkin_LIBRARIES}
        ${PYTHON_LIBRARIES} cnpy caer)

add_executable(neuvisys ${GUI_SOURCES} ${PROJECT_SOURCES} ${ROS_SOURCES})
target_link_libraries(neuvisys PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${catkin_LIBRARIES}
        ${PYTHON_LIBRARIES} cnpy)