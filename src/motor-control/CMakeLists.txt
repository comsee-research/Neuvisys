cmake_minimum_required(VERSION 3.10.0)

project(motor-control)

message("-----------------------------------------------------------------------------------------")
message("Neuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

add_subdirectory(libserie)
include_directories(${catkin_INCLUDE_DIRS})

set(NEUVISYS_MOTOR_SOURCES
        MotorRos.cpp
        BrushlessMotor.cpp
        #../network/Util.cpp
)

add_executable(${PROJECT_NAME} ${NEUVISYS_MOTOR_SOURCES} ${NEUVISYS_NETWORK_SOURCES})
target_link_libraries(${PROJECT_NAME} libserie ${catkin_LIBRARIES} cnpy z)
