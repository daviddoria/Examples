cmake_minimum_required(VERSION 2.6)

PROJECT(PointPlacer)

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(PointPlacer PointPlacer.cxx)
TARGET_LINK_LIBRARIES(PointPlacer vtkHybrid vtkInfovis)


