cmake_minimum_required(VERSION 2.6)

PROJECT(SurfaceFromPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SurfaceFromPoints SurfaceFromPoints.cxx)
TARGET_LINK_LIBRARIES(SurfaceFromPoints vtkHybrid)

ADD_EXECUTABLE(SurfaceFromSpherePoints SurfaceFromSpherePoints.cxx)
TARGET_LINK_LIBRARIES(SurfaceFromSpherePoints vtkHybrid)
