cmake_minimum_required(VERSION 2.6)

PROJECT(Sphere)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DisplaySphere DisplaySphere.cxx)

TARGET_LINK_LIBRARIES(DisplaySphere vtkHybrid)
