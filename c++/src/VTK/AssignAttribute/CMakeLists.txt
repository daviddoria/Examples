cmake_minimum_required(VERSION 2.6)

PROJECT(AssignAttribute)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AssignAttribute AssignAttribute.cxx)
TARGET_LINK_LIBRARIES(AssignAttribute vtkHybrid)
