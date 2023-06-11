cmake_minimum_required(VERSION 3.10)

project(Main VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# TODO: msvc
set(CMAKE_CXX_FLAGS "-O3 -g -Wall")
# -gdwarf-3 -ldl

# setup gtest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
enable_testing()

include(FetchContent)
FetchContent_Declare(
  JetsonGPIO 
  GIT_REPOSITORY https://github.com/pjueon/JetsonGPIO.git 
  GIT_TAG master
)
FetchContent_MakeAvailable(JetsonGPIO)

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

set(VisonLibs pathing config streamer camera arrow)
foreach (VISION_LIB pathing config streamer camera arrow)
    add_library(${VISION_LIB} "src/${VISION_LIB}.cpp")
    target_link_libraries(Vision ${VISION_LIB})
    endforeach()

target_link_libraries(Vision ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(Vision ${OpenCV_LIBS})
target_link_libraries(Vision nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer)

add_library(Controller src/controller.cpp)
target_link_libraries(Controller JetsonGPIO::JetsonGPIO)

add_executable(Main src/main.cpp)
target_link_libraries(Main Controller)
target_link_libraries(Main Vision)

add_executable(Video src/replay.cpp)
target_link_libraries(Video Controller)
target_link_libraries(Video Vision)


add_executable(visionTest src/tests/test_vision.cpp)
target_link_libraries(visionTest GTest::gtest_main)
target_link_libraries(visionTest Vision)
target_link_libraries(visionTest Controller)
include(GoogleTest)
gtest_discover_tests(visionTest)