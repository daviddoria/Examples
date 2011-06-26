cmake_minimum_required(VERSION 2.6)

PROJECT(CustomClassMemberVariable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CustomClassMemberVariable Example.cxx vtkLidarPoint.cxx vtkRay.cxx)
TARGET_LINK_LIBRARIES(CustomClassMemberVariable vtkHybrid)

