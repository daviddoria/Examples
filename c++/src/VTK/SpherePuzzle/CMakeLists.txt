cmake_minimum_required(VERSION 2.6)

PROJECT(SpherePuzzle)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SpherePuzzle SpherePuzzle.cxx)

TARGET_LINK_LIBRARIES(SpherePuzzle vtkHybrid)
