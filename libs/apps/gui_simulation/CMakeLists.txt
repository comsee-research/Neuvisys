set(PROG_NAME neuvisys-simu-qt)

message("-----------------------------------------------------------------------------------------")
message("\t[APP] Neuvisys - ${PROG_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
add_executable(${PROG_NAME} src/main.cpp)
target_link_libraries(${PROG_NAME} PRIVATE network camera motor_control simulator gui_simulation)

install(TARGETS ${PROG_NAME} DESTINATION bin)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
