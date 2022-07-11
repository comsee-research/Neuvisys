set(MY_LIB_NAME motor_control)

message("-----------------------------------------------------------------------------------------")
message("\t[LIB] Neuvisys - ${MY_LIB_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(CATKIN_ENABLE_TESTING OFF)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(SRC_BASE_PATH "src")
set(HDRS_BASE_PATH "include/${MY_LIB_NAME}")

set(NEUVISYS_MOTOR_SOURCES        
		${SRC_BASE_PATH}/BrushlessMotor.cpp
        ${SRC_BASE_PATH}/MotorRos.cpp
)

set(NEUVISYS_MOTOR_HEADERS       
		${HDRS_BASE_PATH}/BrushlessMotor.hpp
		${HDRS_BASE_PATH}/MotorRos.hpp
)

set(NEUVISYS_MOTOR_LIBS
	${catkin_LIBRARIES}
	libserie
)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_library(${MY_LIB_NAME} STATIC ${NEUVISYS_MOTOR_SOURCES} ${NEUVISYS_MOTOR_HEADERS})

target_include_directories(
	${MY_LIB_NAME} 
	PUBLIC  include/
	PROTECTED ${catkin_INCLUDE_DIRS}
	PRIVATE ${libcaer_INCLUDE_DIRS} ${libserie_INCLUDE_DIRS} ${HDRS_BASE_PATH}
)
target_link_libraries(
	${MY_LIB_NAME} 
	PUBLIC ${NEUVISYS_MOTOR_LIBS} 
	PRIVATE network 
)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
