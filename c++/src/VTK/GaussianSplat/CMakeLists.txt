cmake_minimum_required(VERSION 2.6)

PROJECT(GaussianSplat)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GaussianSplat GaussianSplat.cpp)
TARGET_LINK_LIBRARIES(GaussianSplat vtkHybrid)

ADD_EXECUTABLE(GaussianSplatFromFile GaussianSplatFromFile.cpp)
TARGET_LINK_LIBRARIES(GaussianSplatFromFile vtkHybrid)
