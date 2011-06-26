cmake_minimum_required(VERSION 2.6)

PROJECT(ImageClipSimple)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageClipSimple ImageClipSimple.cxx)
TARGET_LINK_LIBRARIES(ImageClipSimple vtkHybrid vtkWidgets)
