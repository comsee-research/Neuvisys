cmake_minimum_required(VERSION 3.10.0)

# Project name and version
project(neuvisys
        VERSION 1.0.0
        LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

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

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)

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

set(PROJECT_SOURCES
        src/SpikingNetwork.cpp
        src/Utils.cpp
        src/Config.cpp
        src/neurons/Neuron.cpp
        src/neurons/SimpleNeuron.cpp
        src/neurons/ComplexNeuron.cpp
)

set(GUI_SOURCES
        src/gui/main.cpp
        src/gui/neuvisysgui.cpp
        src/gui/neuvisysthread.cpp
        src/gui/neuvisysgui.ui)

##### Qt GUI #####
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qt-gui
            ${GUI_SOURCES}
            ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(qt-gui SHARED
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
        )
    else()
        add_executable(qt-gui
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
        )
    endif()
endif()
target_link_libraries(qt-gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${DV_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} /usr/local/lib/libcnpy.so)

##### Neuvisys DV Module #####
add_library(neuvisys-dv SHARED
        src/modules/Neuvisys-DV.cpp
        ${PROJECT_SOURCES})
target_link_libraries(neuvisys-dv PRIVATE ${DV_LIBRARIES} /usr/local/lib/libcnpy.so)
install(TARGETS neuvisys-dv DESTINATION ${DV_MODULES_DIR})

##### Neuvisys Executable #####
add_executable(neuvisys
        src/modules/Neuvisys.cpp
        ${PROJECT_SOURCES})
target_link_libraries(neuvisys ${DV_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} /usr/local/lib/libcnpy.so)

##### Test Executable #####
add_executable(neuvisys-test
        tests/MainTest.cpp
        ${PROJECT_SOURCES})
target_link_libraries(neuvisys-test ${DV_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} /usr/local/lib/libcnpy.so)
