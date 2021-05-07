cmake_minimum_required(VERSION 3.10.0)

# Project name and version
project(neuvisys
        VERSION 1.0.0
        LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

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
#find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)

include_directories ("/usr/include/opencv4/")

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

link_directories(src/dependencies)

set(PROJECT_SOURCES
        src/network/SpikingNetwork.cpp
        src/network/Utils.cpp
        src/network/Config.cpp
        src/network/neurons/Neuron.cpp
        src/network/neurons/SimpleNeuron.cpp
        src/network/neurons/ComplexNeuron.cpp
        src/network/NetworkHandle.cpp)

set(GUI_SOURCES
        src/gui/main.cpp
        src/gui/neuvisysgui.cpp
        src/gui/neuvisysthread.cpp
        src/gui/neuvisysgui.ui)

set_source_files_properties(src/dependencies/*
        PROPERTIES
        COMPILE_OPTIONS "-w")

##### Qt GUI #####
#if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#    qt_add_executable(qt-gui
#            ${GUI_SOURCES}
#            ${PROJECT_SOURCES}
#    )
#else()
#    if(ANDROID)
#        add_library(qt-gui SHARED
#                ${GUI_SOURCES}
#                ${PROJECT_SOURCES}
#        )
#    else()
#        add_executable(qt-gui
#                ${GUI_SOURCES}
#                ${PROJECT_SOURCES}
#        )
#    endif()
#endif()
#target_link_libraries(qt-gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${PYTHON_LIBRARIES} cnpy)

##### Neuvisys DV Module #####
add_library(neuvisys-dv SHARED
        src/modules/NeuvisysDV.cpp
        ${PROJECT_SOURCES})
target_link_libraries(neuvisys-dv PRIVATE ${DV_LIBRARIES} cnpy)
install(TARGETS neuvisys-dv DESTINATION ${DV_MODULES_DIR})

##### Neuvisys Executable #####
add_executable(neuvisys-exe
        src/modules/Neuvisys.cpp
        ${PROJECT_SOURCES})
target_link_libraries(neuvisys-exe ${OpenCV_LIBS} ${PYTHON_LIBRARIES} cnpy)

##### Neuvisys Library #####
#add_library(neuvisys SHARED
#	${PROJECT_SOURCES})
#set_target_properties(neuvisys
#	PROPERTIES VERSION ${PROJECT_VERSION}
#	PUBLIC_HEADER include/neuvisys.h)
#
#configure_file(neuvisys.pc.in neuvisys.pc @ONLY)
#
#target_include_directories(neuvisys PRIVATE .)
#
#install(TARGETS neuvisys
#	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
#install(FILES ${CMAKE_BINARY_DIR}/neuvisys.pc
#	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
