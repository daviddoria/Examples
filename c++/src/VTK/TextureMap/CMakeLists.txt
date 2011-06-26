cmake_minimum_required(VERSION 2.6)

PROJECT(TextureMap)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TextureMap TextureMap.cxx)
TARGET_LINK_LIBRARIES(TextureMap vtkHybrid)

ADD_EXECUTABLE(TextureMapImageData TextureMapImageData.cxx)
TARGET_LINK_LIBRARIES(TextureMapImageData vtkHybrid)
