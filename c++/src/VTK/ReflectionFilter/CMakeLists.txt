cmake_minimum_required(VERSION 2.6)

PROJECT(ReflectionFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReflectionFilter ReflectionFilter.cxx)
TARGET_LINK_LIBRARIES(ReflectionFilter vtkHybrid)
