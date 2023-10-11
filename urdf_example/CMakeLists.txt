cmake_minimum_required(VERSION 3.21)
project(urdf_example LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(camel-tools-trajectory REQUIRED)

#ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/dongju_urdf/")
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(urdf_example urdf_ex.cpp)


target_link_libraries(urdf_example
        raisim::raisim
        pthread
        camel-tools-trajectory)