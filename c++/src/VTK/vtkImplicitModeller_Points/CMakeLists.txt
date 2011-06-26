cmake_minimum_required(VERSION 2.6)
PROJECT(ImplicitModeller_Points)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImplicitModeller_Points ImplicitModeller_Points.cxx)
TARGET_LINK_LIBRARIES(ImplicitModeller_Points vtkHybrid)

