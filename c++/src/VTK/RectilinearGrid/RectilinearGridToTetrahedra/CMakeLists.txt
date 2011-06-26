cmake_minimum_required(VERSION 2.6)

PROJECT(RectilinearGridToTetrahedra)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RectilinearGridToTetrahedra RectilinearGridToTetrahedra.cxx)

TARGET_LINK_LIBRARIES(RectilinearGridToTetrahedra vtkHybrid)
