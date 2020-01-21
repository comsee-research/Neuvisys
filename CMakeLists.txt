# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

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

# Compile this module (standard C++ file, no additional dependencies)
ADD_LIBRARY(neuvisys SHARED Neuvisys.cpp
        src/SpikingNetwork.cpp
        src/SpikingNeuron.cpp
        src/Event.cpp
        src/Utils.cpp)
TARGET_LINK_LIBRARIES(neuvisys PRIVATE ${DV_LIBRARIES} xtensor xtensor::optimize ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

INSTALL(TARGETS neuvisys DESTINATION ${DV_MODULES_DIR})

FIND_PACKAGE(OpenCV REQUIRED)
ADD_EXECUTABLE(test MainTest.cpp
        src/SpikingNetwork.cpp
        src/SpikingNeuron.cpp
        src/Event.cpp
        src/Utils.cpp)
TARGET_LINK_LIBRARIES(test ${OpenCV_LIBS})
