cmake_minimum_required(VERSION 2.6)
PROJECT(Test)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Test Test.cpp)
TARGET_LINK_LIBRARIES(Test vtkHybrid vtkInfovis vtkWidgets vtkGeovis)

