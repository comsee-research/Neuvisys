set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Charts REQUIRED)

set(PROJECT_SOURCES
        ../network/SpikingNetwork.cpp
        ../network/Utils.cpp
        ../network/Config.cpp
        ../network/neurons/Neuron.cpp
        ../network/neurons/SimpleNeuron.cpp
        ../network/neurons/ComplexNeuron.cpp
        ../network/neurons/MotorNeuron.cpp
        ../network/NetworkHandle.cpp)

set(GUI_SOURCES
        main.cpp
        neuvisysgui.cpp
        neuvisysthread.cpp
        neuvisysgui.ui)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(neuvisys-qt
            ${GUI_SOURCES}
            ${PROJECT_SOURCES}
            )
else()
    if(ANDROID)
        add_library(neuvisys-qt SHARED
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                )
    else()
        add_executable(neuvisys-qt
                ${GUI_SOURCES}
                ${PROJECT_SOURCES}
                )
    endif()
endif()
target_link_libraries(neuvisys-qt PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts ${OpenCV_LIBS} ${PYTHON_LIBRARIES} cnpy)