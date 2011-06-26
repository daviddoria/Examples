cmake_minimum_required(VERSION 2.6)
PROJECT(ImageDotProduct)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageDotProduct ImageDotProduct.cxx)
TARGET_LINK_LIBRARIES(ImageDotProduct vtkHybrid)

