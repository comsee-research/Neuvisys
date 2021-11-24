set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
        ${CMAKE_INSTALL_PREFIX}/share/dv /usr/share/dv
        /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

include(dv-modules)
DV_MODULE_SETUP()

set(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../network/Utils.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp
        ../robot-control/motor-control/StepMotor.cpp
        ../robot-control/motor-control/libserie/Faulhaber.cpp
        ../robot-control/motor-control/libserie/comserie.cpp DavisHandle.hpp)

##### Neuvisys DV Module #####
add_library(neuvisys-dv SHARED NeuvisysDV.cpp ${PROJECT_SOURCES})
target_link_libraries(neuvisys-dv PRIVATE ${DV_LIBRARIES} cnpy)
install(TARGETS neuvisys-dv DESTINATION ${DV_MODULES_DIR})

find_package(libcaer)
