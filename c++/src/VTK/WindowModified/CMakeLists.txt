cmake_minimum_required(VERSION 2.6)

PROJECT(WindowModified)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WindowModified WindowModified.cxx)

TARGET_LINK_LIBRARIES(WindowModified vtkHybrid)
