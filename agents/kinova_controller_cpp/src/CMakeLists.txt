# Sources set
LIST(APPEND SOURCES
        ../src/specificworker.cpp
        ../src/api_kinova_controller.cpp
)

# Headers set
LIST(APPEND HEADERS
        ../src/specificworker.h
        ../src/api_kinova_controller.h
        KinovaArm.h
)

SET (LIBS ${LIBS}   dsr_core dsr_gui dsr_api fastcdr fastdds osgDB OpenThreads Eigen3::Eigen QGLViewer-qt6 Qt6OpenGLWidgets)

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t -Wno-unused-variable -Wno-deprecated-declarations -fmax-errors=1 -std=c++2a -fno-char8_t -I/usr/include/x86_64-linux-gnu/qt6/QtOpenGLWidgets/)

link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/../lib/release/libKortexApiCpp.a)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/client)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/messages)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/client_stubs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cxxopts)



