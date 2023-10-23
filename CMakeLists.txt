CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(camel_bipedal LANGUAGES CXX)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)


ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/dongju_urdf/")


find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

find_package(camel-tools-thread REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${RBDL_INC}
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)



include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)


add_executable(camel-bipedal-simul bipedal_demo/bipedalSimul.cpp)
target_link_libraries(camel-bipedal-simul
        PUBLIC
        Qt5::Widgets
        PRIVATE
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
)

add_subdirectory(urdf_example)


