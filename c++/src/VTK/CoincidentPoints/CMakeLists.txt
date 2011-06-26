cmake_minimum_required(VERSION 2.6)

PROJECT(CoincidentPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CoincidentPoints CoincidentPoints.cxx)

TARGET_LINK_LIBRARIES(CoincidentPoints vtkHybrid)
