set(MY_LIB_NAME gui)

message("-----------------------------------------------------------------------------------------")
message("\t[LIB] Neuvisys - ${MY_LIB_NAME}")
message("-----------------------------------------------------------------------------------------")

set(TARGET_NAME "gui")

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
#find_package(libcaer REQUIRED)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(SRC_BASE_PATH "src")
set(HDRS_BASE_PATH "include/${MY_LIB_NAME}")

set(NEUVISYS_GUI_SOURCES        
        ${SRC_BASE_PATH}/Neuvisysgui.cpp
        ${SRC_BASE_PATH}/Neuvisysthread.cpp
        ${SRC_BASE_PATH}/GLWidget.cpp
		${SRC_BASE_PATH}/neuvisysgui.ui
)

set(NEUVISYS_GUI_HEADERS       
		${HDRS_BASE_PATH}/GLWidget.h
        ${HDRS_BASE_PATH}/Neuvisysgui.h
        ${HDRS_BASE_PATH}/Neuvisysthread.h
)

set(NEUVISYS_GUI_LIBS
	Qt${QT_VERSION_MAJOR}::Widgets 
	Qt${QT_VERSION_MAJOR}::Charts
)

set(NEUVISYS_GUI_INCLUDE_DIRS
)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_library(${MY_LIB_NAME} STATIC ${NEUVISYS_GUI_SOURCES} ${NEUVISYS_GUI_HEADERS})

target_include_directories(
	${MY_LIB_NAME} 
	PUBLIC include/
	PROTECTED  
	PRIVATE ${HDRS_BASE_PATH}
)
target_link_libraries(
	${MY_LIB_NAME} 
	PUBLIC ${NEUVISYS_GUI_LIBS}
	PRIVATE network motor_control camera
)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")





##-------------------------------------------------------------------------------
#if (${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#    qt_add_executable(neuvisys-qt
#        ${NEUVISYS_GUI_SOURCES}
#        ${NEUVISYS_CAMERA_SOURCES}
#        ${NEUVISYS_GUI_SOURCES}
#        ${NEUVISYS_MOTOR_SOURCES}
#    )
#else ()
#    if (ANDROID)
#        add_library(neuvisys-qt SHARED 
#                ${NEUVISYS_GUI_SOURCES}
#                ${NEUVISYS_CAMERA_SOURCES}
#                ${NEUVISYS_GUI_SOURCES}
#                ${NEUVISYS_MOTOR_SOURCES}
#                )
#    else ()
#        add_executable(neuvisys-qt Gui.cpp Neuvisysgui.cpp Neuvisysthread.cpp GLWidget.cpp
#                ${NEUVISYS_GUI_SOURCES}
#                ${NEUVISYS_CAMERA_SOURCES}
#                ${NEUVISYS_GUI_SOURCES}
#                ${NEUVISYS_MOTOR_SOURCES}
#                )
#    endif ()
#endif ()


#target_link_libraries(neuvisys-qt PUBLIC ${NEUVISYS_LIBS} ${NEUVISYS_GUI_LIBS} ${NEUVISYS_GUI_LIBS} ${NEUVISYS_CAMERA_LIBS} ${NEUVISYS_MOTOR_LIBS})
#target_include_directories(neuvisys-qt PUBLIC ${NEUVISYS_INCLUDE_DIRS})
