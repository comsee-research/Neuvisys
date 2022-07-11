#=================== includes libs subdirectory ================================
#network
add_subdirectory(network)

#camera + required by gui and gui-simu
if (BUILD_CAMERA OR (BUILD_GUI OR BUILD_GUI_SIMULATION))
    add_subdirectory(camera)
endif ()

#motor-control + required by gui and gui-simu
if (BUILD_MOTOR_CONTROL OR (BUILD_GUI OR BUILD_GUI_SIMULATION))
    add_subdirectory(motor-control)
endif ()

#simulator + required by gui-simu
if (BUILD_SIMULATOR OR BUILD_GUI_SIMULATION)
    add_subdirectory(simulator)
endif ()

#gui
if (BUILD_GUI)
    add_subdirectory(gui)
endif ()

#gui-simu
if (BUILD_GUI_SIMULATION)
    add_subdirectory(gui-simulation)
endif()

#=================== includes apps subdirectory ================================
add_subdirectory(apps)



