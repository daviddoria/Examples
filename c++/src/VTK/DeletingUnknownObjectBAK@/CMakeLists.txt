cmake_minimum_required(VERSION 2.6)

PROJECT(DeletingUnknownObject)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DeletingUnknownObject DeletingUnknownObject.cxx vtkLidarPoint.cxx vtkRay.cxx)
TARGET_LINK_LIBRARIES(DeletingUnknownObject vtkHybrid)

