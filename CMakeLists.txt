cmake_minimum_required(VERSION 3.10)

project(Main VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
# -gdwarf-3 -ldl

# setup gtest
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

# setup opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# setup eigen
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

# setup mjpeg streamer
find_package(nadjieb_mjpeg_streamer REQUIRED)

find_package( Threads )

add_library(Vision src/vision.cpp)
add_library(Controller src/controller.cpp)
add_library(Config src/config.cpp)
add_library(Streamer src/streamer.cpp)
add_executable(Main src/main.cpp)

target_link_libraries(Streamer nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)
target_link_libraries(Vision Streamer)
target_link_libraries(Vision Config)
target_link_libraries(Vision ${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(Main Controller)
target_link_libraries(Main Vision)
target_link_libraries(Main ${OpenCV_LIBS})

add_executable(Video src/replay.cpp)
target_link_libraries(Video Controller)
target_link_libraries(Video Vision)
target_link_libraries(Video ${OpenCV_LIBS})


# TODO: i feel like this isnt how you're meant to do this
add_executable(runTests src/tests/test_vision.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES})
target_link_libraries(runTests ${OpenCV_LIBS})
target_link_libraries(runTests Controller)
target_link_libraries(runTests Streamer)
target_link_libraries(runTests Config)
target_link_libraries(runTests nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)
set_target_properties(runTests PROPERTIES EXCLUDE_FROM_ALL TRUE)
