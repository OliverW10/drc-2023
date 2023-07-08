cmake_minimum_required(VERSION 3.10)

project(Main VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
# -gdwarf-3 -ldl

include(CMakePrintHelpers)

option(ForJetson "" OFF)
option(ForPI "" OFF)

if(ForJetson)
  include(FetchContent)
  FetchContent_Declare(
    JetsonGPIO 
    GIT_REPOSITORY https://github.com/pjueon/JetsonGPIO.git 
    GIT_TAG master
  )
  FetchContent_MakeAvailable(JetsonGPIO)
endif()

if(ForPI)
  # find_package(pigpio CONFIG REQUIRED)
  # fuck cmake
  include_directories(/usr/local/include/)
  # cmake_print_variables(pigpio_INCLUDE_DIR)
endif()

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
  src/car_state.cpp
  src/comm.cpp
  src/config.cpp
  src/controller.cpp
  src/finish_line.cpp
  src/obstacle.cpp
  src/pathing.cpp
  src/streamer.cpp
  src/vision.cpp
)

if(ForPI)
  set(STEER_LIB_SRC src/controllers/steer-pi.cpp)
else()
  set(STEER_LIB_SRC src/controllers/steer-mock.cpp)
endif()

set(ALL_VISION_LIB_SRCS ${LIB_SRCS} ${STEER_LIB_SRC} ${DRIVE_LIB_SRC})

include(CMakePrintHelpers)

cmake_print_variables(ALL_VISION_LIB_SRCS)
add_library(VisionLib ${ALL_VISION_LIB_SRCS})

target_link_libraries(VisionLib ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(VisionLib ${OpenCV_LIBS})
target_link_libraries(VisionLib nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)
if(ForJetson)
  target_link_libraries(VisionLib JetsonGPIO::JetsonGPIO)
endif()
if(ForPI)
  target_link_libraries(VisionLib pigpio)
endif()

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
