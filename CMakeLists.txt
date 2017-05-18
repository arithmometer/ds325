cmake_minimum_required(VERSION 3.5)
project(ds325)

set(CMAKE_CXX_STANDARD 11)

# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

find_package(DepthSense REQUIRED)
include_directories(${DEPTHSENSE_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES src/main.cpp)
add_executable(ds325 ${SOURCE_FILES})
target_link_libraries(ds325 ${DEPTHSENSE_LIBRARIES} ${OpenCV_LIBS})