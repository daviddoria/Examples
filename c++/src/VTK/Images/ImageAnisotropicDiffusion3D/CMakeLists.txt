cmake_minimum_required(VERSION 2.6)

PROJECT(ImageAnisotropicDiffusion3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageAnisotropicDiffusion3D ImageAnisotropicDiffusion3D.cxx)
TARGET_LINK_LIBRARIES(ImageAnisotropicDiffusion3D vtkHybrid)

