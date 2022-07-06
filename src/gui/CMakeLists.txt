cmake_minimum_required(VERSION 3.10.0)

project(neuvisys-qt)

message("-----------------------------------------------------------------------------------------")
message("Neuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

set(TARGET_NAME "neuvisys-qt")


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
find_package(libcaer REQUIRED)

set(NEUVISY_GUI_SOURCES
		src/gui/Gui.cpp
        src/gui/neuvisysgui.ui
        src/gui/Neuvisysgui.cpp
        src/gui/Neuvisysthread.cpp
        src/gui/GLWidget.cpp
)

set(NEUVISYS_GUI_LIBS
	Qt${QT_VERSION_MAJOR}::Widgets 
	Qt${QT_VERSION_MAJOR}::Charts
)

set(NEUVISYS_GUI_INCLUDE_DIRS
)

#-------------------------------------------------------------------------------
if (${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-qt
        ${NEUVISYS_GUI_SOURCES}
        ${NEUVISYS_CAMERA_SOURCES}
        ${NEUVISYS_NETWORK_SOURCES}
        ${NEUVISYS_MOTOR_SOURCES}
    )
else ()
    if (ANDROID)
        add_library(neuvisys-qt SHARED 
                ${NEUVISYS_GUI_SOURCES}
                ${NEUVISYS_CAMERA_SOURCES}
                ${NEUVISYS_NETWORK_SOURCES}
                ${NEUVISYS_MOTOR_SOURCES}
                )
    else ()
        add_executable(neuvisys-qt Gui.cpp Neuvisysgui.cpp Neuvisysthread.cpp GLWidget.cpp
                ${NEUVISYS_GUI_SOURCES}
                ${NEUVISYS_CAMERA_SOURCES}
                ${NEUVISYS_NETWORK_SOURCES}
                ${NEUVISYS_MOTOR_SOURCES}
                )
    endif ()
endif ()


target_link_libraries(neuvisys-qt PUBLIC ${NEUVISYS_LIBS} ${NEUVISYS_GUI_LIBS} ${NEUVISYS_NETWORK_LIBS} ${NEUVISYS_CAMERA_LIBS} ${NEUVISYS_MOTOR_LIBS})
target_include_directories(neuvisys-qt PUBLIC ${NEUVISYS_INCLUDE_DIRS})
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(NEUVISY_GUI_SOURCES ${NEUVISY_GUI_SOURCES} PARENT_SCOPE)
set(NEUVISYS_GUI_LIBS ${NEUVISYS_GUI_LIBS} PARENT_SCOPE)
set(NEUVISYS_GUI_INCLUDE_DIRS ${NEUVISYS_GUI_INCLUDE_DIRS} PARENT_SCOPE)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
