cmake_minimum_required(VERSION 2.6)

PROJECT(ImageCityBlockDistance)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageCityBlockDistance ImageCityBlockDistance.cxx)
TARGET_LINK_LIBRARIES(ImageCityBlockDistance vtkHybrid)

