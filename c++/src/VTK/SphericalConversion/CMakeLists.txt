cmake_minimum_required(VERSION 2.6)

PROJECT(SphericalConversion)

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################

ADD_EXECUTABLE(SphericalConversion SphericalConversion.cpp)

TARGET_LINK_LIBRARIES(SphericalConversion vtkHybrid)
