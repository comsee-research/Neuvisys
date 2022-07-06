cmake_minimum_required(VERSION 3.10.0)
cmake_policy(SET CMP0048 NEW)

# Extract version from header, done first to satisfy CMP0048,
# see `cmake --help-policy CMP0048` for more information.
file(
	STRINGS                         # Read in a file to memory.
	src/version.h           		# File to parse for version number.
	NEUVISYS_PROJECT_VERSION          	# Where to store the results (should just be one line)
	REGEX "#define NEUVISYS_VERSION " 	# The space after is important to ignore 'NEUVISYS_VERSION_INT'
)

# Transform '#define NEUVISYS_VERSION "X.Y.Z"' into 'X.Y.Z'
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" NEUVISYS_PROJECT_VERSION "${NEUVISYS_PROJECT_VERSION}")

# Project name and version
project(neuvisys
        VERSION ${NEUVISYS_PROJECT_VERSION}
        LANGUAGES C CXX)

# Ensure proper build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)

# Options
option(BUILD_CAMERA "Enable to use event based cameras live" OFF)
option(BUILD_SIMULATOR "Enable to use Coppelia simulator" OFF)
option(BUILD_GUI "Enable to use graphical user interface" OFF)
option(BUILD_GUI_SIMULATION "Enable to use graphical user interface with Coppelia simulator" OFF)
option(BUILD_MOTOR_CONTROL "Enable to be able to control motors" OFF)


#================= extra building definitions ==============================
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

add_definitions(
	-pedantic -Wall -Wextra -Werror=format-security 
	-Wshadow -Wnon-virtual-dtor -Wpedantic -Wduplicated-cond -Wuseless-cast
)

#set(GCC_COVERAGE_COMPILE_FLAGS "-03 -D_FORTIFY_SOURCE=2 -D_GLIBCXX_ASSERTIONS -fasynchronous-unwind-tables -fexceptions -fpie -Wl,-pie -fstack-clash-protection -fstack-protector-strong -g -grecord-gcc-switches -pipe -Wall -Werror=format-security -Wl,-z,defs")
#set(GCC_COVERAGE_COMPILE_FLAGS "-O0 -D_GLIBCXX_ASSERTIONS -fasynchronous-unwind-tables -fexceptions -fpie -Wl,-pie -fstack-clash-protection -fstack-protector-strong -g -grecord-gcc-switches -pipe -Wall -Werror=format-security -Wl,-z,defs")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

message("-----------------------------------------------------------------------------------------")
message("${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

#=================== include packages ==========================================
find_package(OpenCV REQUIRED) # 4.0.0 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

#=================== build includes ============================================
#include_directories(${OpenCV_INCLUDE_DIRS}) #include_directories("/usr/include/opencv4/")
#include_directories(${PYTHON_INCLUDE_DIRS})
#include_directories(${HDF5_INCLUDE_DIRS})

include_directories(PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/dependencies)
include(GNUInstallDirs)
link_directories(src/dependencies)
set_source_files_properties(src/dependencies/* PROPERTIES COMPILE_OPTIONS "-w")

#===============================================================================
#=================== build sources and libs ====================================
set(CMAKE_CURRENT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
add_subdirectory(src/network)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/network)

set(NEUVISYS_SOURCES
	${NEUVISYS_NETWORK_SOURCES}
)

set(NEUVISYS_LIBS
	${OpenCV_LIBS} 
	${PYTHON_LIBRARIES} 
	${HDF5_LIBRARIES}
	cnpy
	z
	-lstdc++fs
)

set(NEUVISYS_INCLUDE_DIRS
	src/
	${OpenCV_INCLUDE_DIRS} #include_directories("/usr/include/opencv4/")
	${PYTHON_INCLUDE_DIRS}
	${HDF5_INCLUDE_DIRS}
)

#===============================================================================
#=================== includes subdirectory =====================================
if (BUILD_CAMERA)
    add_subdirectory(src/camera)
endif ()
if (BUILD_SIMULATOR)
    add_subdirectory(src/simulator)
endif ()
if (BUILD_GUI)
    add_subdirectory(src/gui)
endif ()
if (BUILD_MOTOR_CONTROL)
    add_subdirectory(src/motor-control)
endif ()
if (BUILD_GUI_SIMULATION)
    add_subdirectory(src/gui-simulation)
endif()

#=================== build executable ==========================================
message("SOURCES: \n\t${NEUVISYS_SOURCES}\n")
message("LIBS: \n\t${NEUVISYS_LIBS}\n")
message("INCLUDES: \n\t${NEUVISYS_INCLUDE_DIRS}\n")

include_directories(${NEUVISYS_INCLUDE_DIRS})

add_executable(neuvisys-exe src/Neuvisys.cpp ${NEUVISYS_SOURCES})
target_include_directories(neuvisys-exe PRIVATE ${NEUVISYS_INCLUDE_DIRS})
target_link_libraries(neuvisys-exe ${NEUVISYS_LIBS})

message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

