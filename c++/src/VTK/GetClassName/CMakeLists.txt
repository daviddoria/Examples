cmake_minimum_required(VERSION 2.6)

PROJECT(GetClassName)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GetClassName GetClassName.cxx)
TARGET_LINK_LIBRARIES(GetClassName vtkHybrid)
