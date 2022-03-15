set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

include_directories(${catkin_INCLUDE_DIRS})

set(DV_SOURCES
        ../dv-modules/Ynoise.cpp)

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../network/Util.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp)

set(ROS_SOURCES
        ../simulator/SimulationInterface.cpp
        ../simulator/FrameToEvents.cpp
        ../simulator/Motor.cpp
        ../motor-control/BrushlessMotor.cpp
        ../motor-control/libserie/Faulhaber.cpp
        ../motor-control/libserie/comserie.cpp)

set(GUI_SOURCES
        guiMain.cpp
        Neuvisysgui.cpp
        Neuvisysthread.cpp
        GLWidget.cpp
        neuvisysgui.ui)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-qt
            ${DV_SOURCES}
            ${GUI_SOURCES}
            ${PROJECT_SOURCES}
            ${ROS_SOURCES}
            )
else()
    if(ANDROID)
        add_library(neuvisys-qt SHARED
                ${DV_SOURCES}
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${ROS_SOURCES}
                )
    else()
        add_executable(neuvisys-qt
                ${DV_SOURCES}
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${ROS_SOURCES}
                )
    endif()
endif()

target_link_libraries(neuvisys-qt PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${catkin_LIBRARIES}
        ${PYTHON_LIBRARIES} cnpy caer)

add_executable(neuvisys ${DV_SOURCES} ${GUI_SOURCES} ${PROJECT_SOURCES} ${ROS_SOURCES})
target_link_libraries(neuvisys PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${catkin_LIBRARIES}
        ${PYTHON_LIBRARIES} cnpy)