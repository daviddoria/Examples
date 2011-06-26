cmake_minimum_required(VERSION 2.6)

PROJECT(ThinPlateSplineTransform)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ThinPlateSplineTransform ThinPlateSplineTransform.cxx)
TARGET_LINK_LIBRARIES(ThinPlateSplineTransform vtkHybrid)
