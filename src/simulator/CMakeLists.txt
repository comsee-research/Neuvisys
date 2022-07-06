cmake_minimum_required(VERSION 3.10.0)

project(simulator)

message("-----------------------------------------------------------------------------------------")
message("Neuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge)

set(NEUVISYS_SIMULATOR_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/SimulationInterface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Motor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/FrameToEvents.cpp
)

set(NEUVISYS_SIMULATOR_LIBS ${catkin_LIBRARIES})
set(NEUVISYS_SIMULATOR_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

#-------------------------------------------------------------------------------

add_executable(neuvisys-simulator mainSimulator.cpp ${NEUVISYS_SIMULATOR_SOURCES} ${NEUVISYS_NETWORK_SOURCES})
target_include_directories(neuvisys-simulator PUBLIC ${NEUVISYS_SIMULATOR_INCLUDE_DIRS} ${NEUVISYS_INCLUDE_DIRS})
target_link_libraries(neuvisys-simulator ${NEUVISYS_SIMULATOR_LIBS} ${NEUVISYS_LIBS})

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(NEUVISYS_SIMULATOR_SOURCES ${NEUVISYS_SIMULATOR_SOURCES} PARENT_SCOPE)
set(NEUVISYS_SIMULATOR_LIBS ${NEUVISYS_SIMULATOR_LIBS} PARENT_SCOPE)
set(NEUVISYS_SIMULATOR_INCLUDE_DIRS ${NEUVISYS_SIMULATOR_INCLUDE_DIRS} PARENT_SCOPE)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
