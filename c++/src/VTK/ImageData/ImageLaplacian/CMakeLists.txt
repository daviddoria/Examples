cmake_minimum_required(VERSION 2.6)
PROJECT(ImageLaplacian)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageLaplacian ImageLaplacian.cxx)
TARGET_LINK_LIBRARIES(ImageLaplacian vtkHybrid)

