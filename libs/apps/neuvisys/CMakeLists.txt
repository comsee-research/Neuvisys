set(PROG_NAME neuvisys-exe)

message("-----------------------------------------------------------------------------------------")
message("\tNeuvisys - ${PROG_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_executable(${PROG_NAME} src/Neuvisys.cpp)
target_link_libraries(${PROG_NAME} PRIVATE network)
target_include_directories(${PROG_NAME} PRIVATE ${network_INCLUDE_DIRS})

install(TARGETS ${PROG_NAME} DESTINATION bin)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
