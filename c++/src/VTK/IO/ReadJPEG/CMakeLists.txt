cmake_minimum_required(VERSION 2.6)

PROJECT(Read)

#########################################3

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(Read Read.cxx)
TARGET_LINK_LIBRARIES(Read vtkHybrid vtkInfovis)

