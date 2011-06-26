cmake_minimum_required(VERSION 2.6)

PROJECT(ProjectPointPlane)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProjectPointPlane ProjectPointPlane.cxx)
TARGET_LINK_LIBRARIES(ProjectPointPlane vtkHybrid)
