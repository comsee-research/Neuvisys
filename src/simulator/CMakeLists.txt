cmake_minimum_required(VERSION 3.10.0)

project(simulator)

message("-----------------------------------------------------------------------------------------")
message("Neuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package(roscpp)
find_package(libcaer REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${libcaer_INCLUDE_DIRS})

set(NEUVISYS_SIMULATOR_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/SimulationInterface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Motor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FrameToEvents.cpp
PARENT_SCOPE
)

add_executable(neuvisys-simulator mainSimulator.cpp ${NEUVISYS_SIMULATOR_SOURCES} ${NEUVISYS_NETWORK_SOURCES})
target_link_libraries(neuvisys-simulator ${catkin_LIBRARIES} ${NEUVISYS_LIBS} caer)
