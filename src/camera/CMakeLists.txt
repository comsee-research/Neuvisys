cmake_minimum_required(VERSION 3.10.0)

project(camera)

message("-----------------------------------------------------------------------------------------")
message("\tNeuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

find_package(libcaer REQUIRED)

set(NEUVISYS_CAMERA_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/EventCamera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Ynoise.cpp
   PARENT_SCOPE
)
set(NEUVISYS_CAMERA_LIBS libcaer::caer PARENT_SCOPE)
set(NEUVISYS_CAMERA_INCLUDE_DIRS ${libcaer_INCLUDE_DIRS} PARENT_SCOPE)
message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
