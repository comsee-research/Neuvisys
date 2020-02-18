# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

set(CMAKE_CXX_STANDARD 17)
set(GCC_COVERAGE_COMPILE_FLAGS "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

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

# Project name and version
PROJECT(neuvisys-dv
        VERSION 1.0.0
        LANGUAGES C CXX)

find_package(xtensor REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Custom CMake base module
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
        ${CMAKE_INSTALL_PREFIX}/share/dv /usr/share/dv
        /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

# Basic setup
INCLUDE(dv-modules)
INCLUDE_DIRECTORIES(${xtensor_INCLUDE_DIRS})
DV_MODULE_SETUP()

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

##### Neuvisys Module #####
ADD_LIBRARY(neuvisys SHARED dv-modules/Neuvisys.cpp
        src/SpikingNetwork.cpp
        src/neurons/Neuron.cpp
        src/neurons/TemporalNeuron.cpp
        src/neurons/SpatioTemporalNeuron.cpp
        src/neurons/OrientedNeuron.cpp
        src/Utils.cpp
        src/Config.cpp)
TARGET_LINK_LIBRARIES(neuvisys PRIVATE ${DV_LIBRARIES} xtensor xtensor::optimize ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

INSTALL(TARGETS neuvisys DESTINATION ${DV_MODULES_DIR})

##### EventAnalysis Module #####

ADD_LIBRARY(event-analysis SHARED dv-modules/EventAnalysis.cpp)
TARGET_LINK_LIBRARIES(event-analysis PRIVATE ${DV_LIBRARIES})
INSTALL(TARGETS event-analysis DESTINATION ${DV_MODULES_DIR})

##### Tests Executables #####
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

ADD_EXECUTABLE(neuvisys-test tests/MainTest.cpp
        src/SpikingNetwork.cpp
        src/neurons/Neuron.cpp
        src/neurons/TemporalNeuron.cpp
        src/neurons/SpatioTemporalNeuron.cpp
        src/neurons/OrientedNeuron.cpp
        src/Utils.cpp
        src/Config.cpp)
TARGET_LINK_LIBRARIES(neuvisys-test ${DV_LIBRARIES} xtensor xtensor::optimize ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
