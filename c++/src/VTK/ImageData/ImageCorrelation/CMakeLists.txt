cmake_minimum_required(VERSION 2.6)
PROJECT(ImageCorrelation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageCorrelation ImageCorrelation.cxx)
TARGET_LINK_LIBRARIES(ImageCorrelation vtkHybrid vtkWidgets)

