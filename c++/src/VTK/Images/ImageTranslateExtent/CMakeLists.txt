cmake_minimum_required(VERSION 2.6)

PROJECT(ImageTranslateExtent)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageTranslateExtent ImageTranslateExtent.cxx)
TARGET_LINK_LIBRARIES(ImageTranslateExtent vtkHybrid)

