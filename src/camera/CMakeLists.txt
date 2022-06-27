find_package(libcaer REQUIRED)

set(SOURCES
        EventCamera.cpp
        Ynoise.cpp)

add_executable(event-camera ${SOURCES})
target_link_libraries(event-camera caer)
