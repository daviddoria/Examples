cmake_minimum_required(VERSION 2.6)

PROJECT(ImageNonMaximumSuppression)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageNonMaximumSuppression ImageNonMaximumSuppression.cxx)
TARGET_LINK_LIBRARIES(ImageNonMaximumSuppression vtkHybrid)
