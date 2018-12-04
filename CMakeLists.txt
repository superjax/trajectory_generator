cmake_minimum_required(VERSION 3.4)
project(TrajectoryGenerator)

find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

if (${GTEST_NOTFOUND})
    add_subdirectory(lib/googletest)
endif()

add_subdirectory(lib/geometry)
add_subdirectory(lib/mav_trajectory_generation/mav_trajectory_generation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include
    ${EIGEN3_INCLUDE_DIRS}
    lib/geometry/include
    lib/mav_trajectory_generation/mav_trajectory_generation/include)

add_library(TrajOpt
    src/trajectory.cpp
    include/trajectory.h)
target_link_libraries(TrajOpt
    mav_trajectory_generation
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
    Qt5::Gui)

add_executable(TrajectoryGenerator
  src/main.cpp
  src/mainwindow.cpp
  src/scribblearea.cpp
  include/mainwindow.h
  include/scribblearea.h
)
target_link_libraries(TrajectoryGenerator
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
    Qt5::Gui
    TrajOpt
)

add_executable(TrajOpt_Test src/trajopt_test.cpp)
target_link_libraries(TrajOpt_Test
    gtest
    gtest_main
    ${GTEST_LIBRARIES}
    TrajOpt
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
    Qt5::Gui
    ${CERES_LIBRARIES}
    pthread
)


