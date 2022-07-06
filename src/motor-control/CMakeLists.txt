cmake_minimum_required(VERSION 3.10.0)

project(motor-control)

message("-----------------------------------------------------------------------------------------")
message("\tNeuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
add_subdirectory(libserie)

set(NEUVISYS_MOTOR_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/BrushlessMotor.cpp
)

set(NEUVISYS_MOTOR_LIBS libserie ${catkin_LIBRARIES})
set(NEUVISYS_MOTOR_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

#-------------------------------------------------------------------------------
add_executable(motor-control MotorRos.cpp ${NEUVISYS_MOTOR_SOURCES} ${NEUVISYS_NETWORK_SOURCES})
target_include_directories(motor-control PUBLIC ${NEUVISYS_MOTOR_INCLUDE_DIRS} ${NEUVISYS_INCLUDE_DIRS})
target_link_libraries(motor-control ${NEUVISYS_MOTOR_LIBS} ${NEUVISYS_LIBS})

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(NEUVISYS_MOTOR_SOURCES ${NEUVISYS_MOTOR_SOURCES} PARENT_SCOPE)
set(NEUVISYS_LIBSERIE_SOURCES ${NEUVISYS_LIBSERIE_SOURCES} PARENT_SCOPE)

set(NEUVISYS_MOTOR_LIBS ${NEUVISYS_MOTOR_LIBS} PARENT_SCOPE)
set(NEUVISYS_MOTOR_INCLUDE_DIRS ${NEUVISYS_MOTOR_INCLUDE_DIRS} PARENT_SCOPE)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
