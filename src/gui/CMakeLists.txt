set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)
find_package(libcaer)

set(CAMERA_SOURCES
        ../camera/EventCamera.cpp
        ../camera/Ynoise.cpp)

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../resources/DefaultConfig.cpp
        ../utils/Util.cpp
        ../utils/WeightMatrix.cpp
        ../utils/WeightMap.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp
        ../network/SurroundSuppression.cpp)

set(MOTOR_SOURCES
        ../motor-control/BrushlessMotor.cpp
        ../motor-control/libserie/Faulhaber.cpp
        ../motor-control/libserie/comserie.cpp)

set(GUI_SOURCES
        Gui.cpp
        Neuvisysgui.cpp
        Neuvisysthread.cpp
        GLWidget.cpp
        neuvisysgui.ui)

if (${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-qt
            ${CAMERA_SOURCES}
            ${GUI_SOURCES}
            ${PROJECT_SOURCES}
            ${MOTOR_SOURCES}
            )
else ()
    if (ANDROID)
        add_library(neuvisys-qt SHARED
                ${CAMERA_SOURCES}
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${MOTOR_SOURCES}
                )
    else ()
        add_executable(neuvisys-qt
                ${CAMERA_SOURCES}
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                ${MOTOR_SOURCES}
                )
    endif ()
endif ()

target_link_libraries(neuvisys-qt PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${HDF5_LIBRARIES} cnpy caer z)
