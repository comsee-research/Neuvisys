cmake_minimum_required(VERSION 3.0)

project(StepMotor)

add_subdirectory(libserie)

set(SRCS StepMotor.cpp)

add_executable(stepmotor ${SRCS})
target_link_libraries(stepmotor libserie ${catkin_LIBRARIES})
