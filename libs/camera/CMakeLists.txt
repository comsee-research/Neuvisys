set(MY_LIB_NAME camera)

message("-----------------------------------------------------------------------------------------")
message("\t[LIB] Neuvisys - ${MY_LIB_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
find_package(libcaer REQUIRED)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(SRC_BASE_PATH "src")
set(HDRS_BASE_PATH "include/${MY_LIB_NAME}")

set(NEUVISYS_CAMERA_SOURCES        
		${SRC_BASE_PATH}/EventCamera.cpp
        ${SRC_BASE_PATH}/Ynoise.cpp
)

set(NEUVISYS_CAMERA_HEADERS       
		${HDRS_BASE_PATH}/EventCamera.hpp
		${HDRS_BASE_PATH}/Ynoise.hpp
)

set(NEUVISYS_CAMERA_LIBS
	libcaer::caer
)

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_library(${MY_LIB_NAME} STATIC ${NEUVISYS_CAMERA_SOURCES} ${NEUVISYS_CAMERA_HEADERS})

target_include_directories(
	${MY_LIB_NAME} 
	PUBLIC ${libcaer_INCLUDE_DIRS} include/
	PROTECTED ${libcaer_INCLUDE_DIRS}
)
target_link_libraries(
	${MY_LIB_NAME} 
	PUBLIC ${NEUVISYS_CAMERA_LIBS}
	PRIVATE network
)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
