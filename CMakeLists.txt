cmake_minimum_required(VERSION 3.4)
project(TrajectoryGenerator)

FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5PrintSupport)
#FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5OpenGL REQUIRED)
FIND_PACKAGE(GTest)
FIND_PACKAGE(Eigen3 REQUIRED)

if (${GTEST_NOTFOUND})
    add_subdirectory(lib/googletest)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

include_directories(include
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  src/scribblearea.cpp

  include/mainwindow.h
  include/scribblearea.h
)
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::OpenGL
)
