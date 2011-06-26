cmake_minimum_required(VERSION 2.6)

PROJECT(LabeledDataMapperPointIDs)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LabeledDataMapperPointIDs LabeledDataMapperPointIDs.cxx)

TARGET_LINK_LIBRARIES(LabeledDataMapperPointIDs vtkHybrid)
