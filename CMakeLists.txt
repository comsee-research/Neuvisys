cmake_minimum_required(VERSION 3.10.0)

# Project name and version
project(neuvisys
        VERSION 1.0.0
        LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

# General build settings
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel")
ENDIF()

IF (NOT CMAKE_INSTALL_PREFIX)
    IF (UNIX AND NOT APPLE)
        SET(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake default install prefix, set to /usr on Unix/Linux")
    ELSEIF(APPLE)
        SET(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "CMake default install prefix, set to /usr/local on macOS")
    ELSE()
        MESSAGE(FATAL_ERROR "CMAKE_INSTALL_PREFIX is not set")
    ENDIF()
ENDIF()

find_package(OpenCV 4.0.0 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories ("/usr/include/opencv4/")
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/dependencies)
include(GNUInstallDirs)
link_directories(src/dependencies)
set_source_files_properties(src/dependencies/* PROPERTIES COMPILE_OPTIONS "-w")

set(PROJECT_SOURCES
        src/network/SpikingNetwork.cpp
        src/network/Utils.cpp
        src/network/Config.cpp
        src/network/neurons/Neuron.cpp
        src/network/neurons/SimpleNeuron.cpp
        src/network/neurons/ComplexNeuron.cpp
        src/network/neurons/MotorNeuron.cpp
        src/network/NetworkHandle.cpp)

add_executable(neuvisys-exe src/Neuvisys.cpp ${PROJECT_SOURCES})

target_link_libraries(neuvisys-exe ${OpenCV_LIBS} ${PYTHON_LIBRARIES} cnpy)

#add_subdirectory(src/gui)
#add_subdirectory(src/dv-modules)
#add_subdirectory(src/ros)
