cmake_minimum_required(VERSION 3.10)

project(Main VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# TODO: msvc
set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
# -gdwarf-3 -ldl


# include(FetchContent)
# FetchContent_Declare(
#   JetsonGPIO 
#   GIT_REPOSITORY https://github.com/pjueon/JetsonGPIO.git 
#   GIT_TAG master
# )
# FetchContent_MakeAvailable(JetsonGPIO)

# setup opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# setup eigen
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

# setup mjpeg streamer
find_package(nadjieb_mjpeg_streamer REQUIRED)

find_package( Threads )

set(LIB_SRCS
  src/arrow.cpp
  src/camera.cpp
  src/comm.cpp
  src/config.cpp
  src/controller.cpp
  src/obstacle.cpp
  src/pathing.cpp
  src/streamer.cpp
  src/vision.cpp
)
add_library(VisionLib ${LIB_SRCS})

target_link_libraries(VisionLib ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(VisionLib ${OpenCV_LIBS})
target_link_libraries(VisionLib nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)
# target_link_libraries(VisionLib JetsonGPIO::JetsonGPIO)

add_executable(Main src/main.cpp)
target_link_libraries(Main VisionLib)

add_executable(Video src/replay.cpp)
target_link_libraries(Video VisionLib)

if(DEFINED DO_UNIT_TEST)
  # setup gtest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  FetchContent_MakeAvailable(googletest)
  enable_testing()

  add_executable(unit-test src/tests/test_vision.cpp)
  target_link_libraries(unit-test GTest::gtest_main)
  target_link_libraries(unit-test VisionLib)
  include(GoogleTest)
  gtest_discover_tests(unit-test)
  set_target_properties(unit-test PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()
