cmake_minimum_required(VERSION 2.6)

PROJECT(ExtraData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtraData ExtraData.cpp)
TARGET_LINK_LIBRARIES(ExtraData vtkHybrid)

ADD_EXECUTABLE(ReadAllData ReadAllData.cpp)
TARGET_LINK_LIBRARIES(ReadAllData vtkHybrid)

ADD_EXECUTABLE(DetermineDataType DetermineDataType.cpp)
TARGET_LINK_LIBRARIES(DetermineDataType vtkHybrid)