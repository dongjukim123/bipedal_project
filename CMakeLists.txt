CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(camel_bipedal LANGUAGES CXX)

set(RAISIM_PATH "$ENV{HOME}/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)


ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/dongju_urdf/")


find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

find_package(RBDL COMPONENTS URDFREADER REQUIRED)



include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)


add_subdirectory(urdf_example)


