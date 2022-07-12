# 'Google_test' is the subproject name
project(tests)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

set(TEST_SOURCES
        ../src/utils/Util.cpp
        ../src/utils/WeightMatrix.cpp
        ../src/utils/WeightMap.cpp
        ../src/resources/DefaultConfig.cpp
        ../src/network/SpikingNetwork.cpp
        ../src/network/Config.cpp
        ../src/network/neurons/Neuron.cpp
        ../src/network/neurons/SimpleNeuron.cpp
        ../src/network/neurons/ComplexNeuron.cpp
        ../src/network/neurons/MotorNeuron.cpp
        ../src/network/NetworkHandle.cpp)

set(FIXTURE_SOURCES
        SpikingNetworkTest.cpp
        NetworkHandleTest.cpp)


add_executable(neuvisys_tests ${FIXTURE_SOURCES} ${TEST_SOURCES})
target_link_libraries(neuvisys_tests gtest gtest_main ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${HDF5_LIBRARIES} cnpy z)