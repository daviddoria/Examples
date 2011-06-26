cmake_minimum_required(VERSION 2.6)
PROJECT(ImageData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageData ImageData.cxx)
TARGET_LINK_LIBRARIES(ImageData vtkHybrid)

ADD_EXECUTABLE(MultipleComponents MultipleComponents.cxx)
TARGET_LINK_LIBRARIES(MultipleComponents vtkHybrid)

ADD_EXECUTABLE(Colors Colors.cxx)
TARGET_LINK_LIBRARIES(Colors vtkHybrid)
