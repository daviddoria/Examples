cmake_minimum_required(VERSION 2.6)

PROJECT(CastImageFilter)

#include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

#FIND_PACKAGE(VTK REQUIRED)
#INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CastImageFilter CastImageFilter.cxx)
TARGET_LINK_LIBRARIES(CastImageFilter ${ITK_LIBRARIES})

ADD_EXECUTABLE(CastImageFilter_Vector CastImageFilter_Vector.cxx)
TARGET_LINK_LIBRARIES(CastImageFilter_Vector ${ITK_LIBRARIES})