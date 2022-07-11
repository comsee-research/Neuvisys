set(MY_LIB_NAME simulator)

message("-----------------------------------------------------------------------------------------")
message("\t[LIB] Neuvisys - ${MY_LIB_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(CATKIN_ENABLE_TESTING OFF)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(SRC_BASE_PATH "src")
set(HDRS_BASE_PATH "include/${MY_LIB_NAME}")

set(NEUVISYS_SIMU_SOURCES        
		${SRC_BASE_PATH}/SimulationInterface.cpp
        ${SRC_BASE_PATH}/Motor.cpp
        ${SRC_BASE_PATH}/FrameToEvents.cpp
)

set(NEUVISYS_SIMU_HEADERS       
		${HDRS_BASE_PATH}/SimulationInterface.hpp
		${HDRS_BASE_PATH}/Motor.hpp
		${HDRS_BASE_PATH}/FrameToEvents.hpp
)

set(NEUVISYS_SIMU_LIBS
	${catkin_LIBRARIES}
)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_library(${MY_LIB_NAME} STATIC ${NEUVISYS_SIMU_SOURCES} ${NEUVISYS_SIMU_HEADERS})

target_include_directories(
	${MY_LIB_NAME} 
	PUBLIC  include/ coppelia-scenes/
	PROTECTED ${catkin_INCLUDE_DIRS}
	PRIVATE ${libcaer_INCLUDE_DIRS} ${HDRS_BASE_PATH}
)
target_link_libraries(
	${MY_LIB_NAME} 
	PUBLIC ${NEUVISYS_SIMU_LIBS} 
	PRIVATE network 
)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

