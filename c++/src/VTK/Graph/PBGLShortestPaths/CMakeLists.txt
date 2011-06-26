cmake_minimum_required(VERSION 2.6)

PROJECT(PBGLShortestPaths)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PBGLShortestPaths PBGLShortestPaths.cxx)
TARGET_LINK_LIBRARIES(PBGLShortestPaths vtkHybrid vtkInfovis vtkParallel)

