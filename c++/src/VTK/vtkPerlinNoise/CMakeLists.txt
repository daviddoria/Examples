cmake_minimum_required(VERSION 2.6)
PROJECT(PerlinNoise)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PerlinNoise PerlinNoise.cxx)
TARGET_LINK_LIBRARIES(PerlinNoise vtkHybrid)

