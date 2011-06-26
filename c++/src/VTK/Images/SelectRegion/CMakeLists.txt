cmake_minimum_required(VERSION 2.6)

PROJECT(SelectRegion)

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(SelectRegion SelectRegion.cxx)
TARGET_LINK_LIBRARIES(SelectRegion vtkHybrid)


