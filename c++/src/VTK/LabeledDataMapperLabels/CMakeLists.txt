cmake_minimum_required(VERSION 2.6)

PROJECT(LabeledDataMapperLabels)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LabeledDataMapperLabels LabeledDataMapperLabels.cxx)

TARGET_LINK_LIBRARIES(LabeledDataMapperLabels vtkHybrid)
