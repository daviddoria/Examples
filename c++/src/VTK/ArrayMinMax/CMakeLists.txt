cmake_minimum_required(VERSION 2.6)

PROJECT(ArrayMinMax)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayMinMax ArrayMinMax.cxx)
TARGET_LINK_LIBRARIES(ArrayMinMax vtkHybrid)

