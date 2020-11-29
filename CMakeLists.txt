# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

set(CMAKE_CXX_STANDARD 17)
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

# Project name and version
project(neuvisys-dv
        VERSION 1.0.0
        LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

# Define installation paths
include(GNUInstallDirs)

# Custom CMake base module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
        ${CMAKE_INSTALL_PREFIX}/share/dv /usr/share/dv
        /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

# Basic setup
include(dv-modules)
include_directories(${PYTHON_INCLUDE_DIRS})
DV_MODULE_SETUP()

# Set full RPATH, modules are libraries for DV
set(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

##### Neuvisys Module #####
add_library(neuvisys SHARED
        dv-modules/Neuvisys.cpp
        src/SpikingNetwork.cpp
        src/neurons/Neuron.cpp
        src/neurons/SimpleNeuron.cpp
        src/neurons/ComplexNeuron.cpp
        src/Utils.cpp
        src/Config.cpp)
target_link_libraries(neuvisys PRIVATE ${DV_LIBRARIES} /usr/local/lib/libcnpy.so)
install(TARGETS neuvisys DESTINATION ${DV_MODULES_DIR})

##### Test Executable #####
add_executable(neuvisys-test
        tests/MainTest.cpp
        src/SpikingNetwork.cpp
        src/neurons/Neuron.cpp
        src/neurons/SimpleNeuron.cpp
        src/neurons/ComplexNeuron.cpp
        src/Utils.cpp
        src/Config.cpp)
target_link_libraries(neuvisys-test ${DV_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} /usr/local/lib/libcnpy.so)

##### EventAnalysis Executable #####
add_executable(event-analysis
        dv-modules/EventAnalysis.cpp
        src/SpikingNetwork.cpp
        src/neurons/Neuron.cpp
        src/neurons/SimpleNeuron.cpp
        src/neurons/ComplexNeuron.cpp
        src/Utils.cpp
        src/Config.cpp)
target_link_libraries(event-analysis ${DV_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} /usr/local/lib/libcnpy.so)
