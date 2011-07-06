cmake_minimum_required(VERSION 2.6)

PROJECT(DeformationFieldSource)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(DeformationFieldSource DeformationFieldSource.cxx)
TARGET_LINK_LIBRARIES(DeformationFieldSource ${ITK_LIBRARIES})
