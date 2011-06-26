cmake_minimum_required(VERSION 2.6)

PROJECT(BackfaceCulling)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BackfaceCulling BackfaceCulling.cxx)
TARGET_LINK_LIBRARIES(BackfaceCulling vtkHybrid)
