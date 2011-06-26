cmake_minimum_required(VERSION 2.6)
PROJECT(ImplicitModeller)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImplicitModeller ImplicitModeller.cxx)
TARGET_LINK_LIBRARIES(ImplicitModeller vtkHybrid vtkVolumeRendering)

