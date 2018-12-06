cmake_minimum_required(VERSION 3.4)
project(trajectory_generator)

find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosflight_msgs
    nav_msgs
    geometry_msgs
    message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  TrajectoryCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

if (${GTEST_NOTFOUND})
    add_subdirectory(lib/googletest)
endif()

add_subdirectory(lib/geometry)
add_subdirectory(lib/mav_trajectory_generation/mav_trajectory_generation)
add_subdirectory(lib/lin_alg_tools)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include
    ${EIGEN3_INCLUDE_DIRS}
    lib/geometry/include
    lib/mav_trajectory_generation/mav_trajectory_generation/include
    lib/lin_alg_tools/include
    ${catkin_INCLUDE_DIRS})

add_executable(lqr
    src/lqr.cpp
    include/lqr.h

    src/lqr_ros.cpp
    include/lqr_ros.h
)
target_link_libraries(lqr
    lin_alg_tools
    ${catkin_LIBRARIES})

add_executable(TrajectoryGenerator
  src/main.cpp
  src/mainwindow.cpp
  src/scribblearea.cpp
  include/mainwindow.h
  include/scribblearea.h

  src/trajectory.cpp
  include/trajectory.h

  src/trajopt_ros.cpp
  include/trajopt_ros.h
)
target_link_libraries(TrajectoryGenerator
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
    Qt5::Gui
    mav_trajectory_generation
    ${catkin_LIBRARIES}
)
add_dependencies(TrajectoryGenerator ${catkin_EXPORTED_TARGETS})

#add_executable(TrajOpt_Test src/trajopt_test.cpp)
#target_link_libraries(TrajOpt_Test
#    gtest
#    gtest_main
#    ${GTEST_LIBRARIES}
#    TrajOpt
#    Qt5::Widgets
#    Qt5::PrintSupport
#    Qt5::OpenGL
#    Qt5::Gui
#    ${CERES_LIBRARIES}
#    pthread
#)
