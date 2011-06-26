cmake_minimum_required(VERSION 2.6)

PROJECT(ShadowMapPass)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShadowMapPass ShadowMapPass.cxx)
TARGET_LINK_LIBRARIES(ShadowMapPass vtkHybrid)
