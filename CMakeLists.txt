cmake_minimum_required(VERSION 3.10)

project(Main VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(CMakePrintHelpers)
find_package(PkgConfig REQUIRED)

# setup gstreamer
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS})
# target_compile_options(testapp PUBLIC ${SDL2_CFLAGS_OTHER})
# message(GSTREAMER_LIBRARY="${GSTREAMER_LIBRARY}")
# message(GSTREAMER_INCLUDE="${GSTREAMER_INCLUDE}/gstreamer-1.0")
# include_directories("${GSTREAMER_INCLUDE}/gstreamer-1.0")

# setup opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# setup eigen
find_package( Eigen3 REQUIRED )
cmake_print_variables(EIGEN3_INCLUDE_DIR)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(Vision src/vision.cpp)
add_library(Controller src/controller.cpp)
add_executable(Main src/main.cpp)

target_link_libraries(Main Controller)
target_link_libraries(Main Vision)
target_link_libraries(Main ${OpenCV_LIBS})
target_link_libraries(Main ${GSTREAMER_LIBRARIES})

# -DCMAKE_EXPORT_COMPILE_COMMANDS=ON to generate compile_commands for vscode