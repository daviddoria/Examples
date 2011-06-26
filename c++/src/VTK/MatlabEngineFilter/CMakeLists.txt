cmake_minimum_required(VERSION 2.6)
PROJECT(MatlabEngineFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MatlabEngineFilter MatlabEngineFilter.cxx)
TARGET_LINK_LIBRARIES(MatlabEngineFilter vtkHybrid)

