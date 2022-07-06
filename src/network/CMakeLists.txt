cmake_minimum_required(VERSION 3.10.0)

project(network)

message("-----------------------------------------------------------------------------------------")
message("\tNeuvisys - ${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
set(NEUVISYS_NETWORK_SOURCES        
		${CMAKE_CURRENT_SOURCE_DIR}/SpikingNetwork.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Util.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/NetworkHandle.cpp
        #---------
        ${CMAKE_CURRENT_SOURCE_DIR}/neurons/Neuron.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/neurons/SimpleNeuron.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/neurons/ComplexNeuron.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/neurons/MotorNeuron.cpp
   PARENT_SCOPE
)
set(NEUVISYS_NETWORK_LIBS PARENT_SCOPE)
set(NEUVISYS_NETWORK_INCLUDE_DIRS PARENT_SCOPE)

message("\n\t-- Done.\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
