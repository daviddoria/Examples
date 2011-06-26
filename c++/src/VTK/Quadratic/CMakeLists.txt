cmake_minimum_required(VERSION 2.6)

PROJECT(Quadratic)

set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

###############
ADD_EXECUTABLE(Quadratic Quadratic.cpp)
TARGET_LINK_LIBRARIES(Quadratic vtkHybrid)

###############
ADD_EXECUTABLE(QuadraticFile QuadraticFile.cpp)
TARGET_LINK_LIBRARIES(QuadraticFile vtkHybrid)