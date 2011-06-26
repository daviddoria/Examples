cmake_minimum_required(VERSION 2.6)

PROJECT(SmartPointers)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SmartPointers SmartPointers.cpp)

TARGET_LINK_LIBRARIES(SmartPointers vtkHybrid)
