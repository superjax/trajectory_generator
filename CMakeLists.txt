cmake_minimum_required(VERSION 3.4)
project(TrajectoryGenerator)

find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
#find_package(Qt5Gui)
find_package(Qt5OpenGL REQUIRED)
find_package(GTest)
find_package(Eigen3 REQUIRED)

if (${GTEST_NOTFOUND})
    add_subdirectory(lib/googletest)
endif()

add_subdirectory(lib/geometry)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include
    ${EIGEN3_INCLUDE_DIRS}
    lib/geometry/include)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  src/scribblearea.cpp
  src/trajectory.cpp
  src/multirotor.cpp

  include/mainwindow.h
  include/scribblearea.h
  include/trajectory.h
  include/multirotor.h
)
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
)
