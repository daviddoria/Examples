cmake_minimum_required(VERSION 2.6)

PROJECT(CenterImage)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CenterImage CenterImage.cxx)
TARGET_LINK_LIBRARIES(CenterImage vtkHybrid vtkWidgets)
