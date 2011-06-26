cmake_minimum_required(VERSION 2.6)

PROJECT(VTK)
include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
#include_directories(${ITK_SOURCE_DIR}/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VTK VTK.cpp)
TARGET_LINK_LIBRARIES(VTK
vtkHybrid
ITKBasicFilters ITKCommon ITKIO
)

#ADD_EXECUTABLE(Mesh Mesh.cpp)
#TARGET_LINK_LIBRARIES(Mesh
#vtkHybrid
#ITKBasicFilters ITKCommon ITKIO
#)

