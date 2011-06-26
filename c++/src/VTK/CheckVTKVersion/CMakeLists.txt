cmake_minimum_required(VERSION 2.6)

PROJECT(CheckVTKVersion)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CheckVTKVersion CheckVTKVersion.cxx)
TARGET_LINK_LIBRARIES(CheckVTKVersion vtkHybrid)
