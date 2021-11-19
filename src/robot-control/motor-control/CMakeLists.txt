find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

add_subdirectory(libserie)
include_directories(${catkin_INCLUDE_DIRS})

set(SRCS StepMotorRos.cpp StepMotor.cpp)

add_executable(stepmotor ${SRCS})
target_link_libraries(stepmotor libserie ${catkin_LIBRARIES})
