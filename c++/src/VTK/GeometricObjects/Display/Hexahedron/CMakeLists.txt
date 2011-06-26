cmake_minimum_required(VERSION 2.6)

PROJECT(Hexahedron)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Hexahedron Hexahedron.cxx)

TARGET_LINK_LIBRARIES(Hexahedron vtkHybrid)
