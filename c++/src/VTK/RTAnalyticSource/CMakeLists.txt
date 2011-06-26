cmake_minimum_required(VERSION 2.6)

PROJECT(RTAnalyticSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RTAnalyticSource RTAnalyticSource.cxx)
TARGET_LINK_LIBRARIES(RTAnalyticSource vtkHybrid)
