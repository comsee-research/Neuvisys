find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

include_directories(${catkin_INCLUDE_DIRS})

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../network/Utils.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp
        Motor.cpp
        FrameToEvents.cpp)

add_executable(neuvisys-ros SimulationInterface.cpp ${PROJECT_SOURCES})
target_link_libraries(neuvisys-ros ${catkin_LIBRARIES} ${OpenCV_LIBS} cnpy)