cmake_minimum_required(VERSION 2.6)
PROJECT(LinearExtrusion)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LinearExtrusion LinearExtrusion.cxx)
TARGET_LINK_LIBRARIES(LinearExtrusion vtkHybrid)

