cmake_minimum_required(VERSION 3.10)
PROJECT(bipedal_gui LANGUAGES CXX)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME} STATIC
            include/bipedal_gui/mainwindow.cpp)

target_link_libraries(${PROJECT_NAME}
            Qt5::Widgets
            )