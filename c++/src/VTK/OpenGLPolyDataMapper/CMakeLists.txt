cmake_minimum_required(VERSION 2.6)
PROJECT(OpenGLPolyDataMapper)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OpenGLPolyDataMapper OpenGLPolyDataMapper.cxx)
TARGET_LINK_LIBRARIES(OpenGLPolyDataMapper vtkHybrid)

