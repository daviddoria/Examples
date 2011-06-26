cmake_minimum_required(VERSION 2.6)

PROJECT(ImageAnisotropicDiffusion2D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageAnisotropicDiffusion2D ImageAnisotropicDiffusion2D.cxx)
TARGET_LINK_LIBRARIES(ImageAnisotropicDiffusion2D vtkHybrid)

