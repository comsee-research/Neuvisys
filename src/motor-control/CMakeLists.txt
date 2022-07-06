find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

add_subdirectory(libserie)
include_directories(${catkin_INCLUDE_DIRS})

set(SRCS
        MotorRos.cpp
        BrushlessMotor.cpp
        ../utils/Util.cpp)

add_executable(motor-control ${SRCS})
target_link_libraries(motor-control libserie ${catkin_LIBRARIES} cnpy z)
