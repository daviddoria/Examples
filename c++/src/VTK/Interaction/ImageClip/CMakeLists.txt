cmake_minimum_required(VERSION 2.6)

PROJECT(ImageClip)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageClip ImageClip.cxx)
TARGET_LINK_LIBRARIES(ImageClip vtkHybrid vtkWidgets)
