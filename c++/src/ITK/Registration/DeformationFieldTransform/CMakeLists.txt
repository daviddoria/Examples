cmake_minimum_required(VERSION 2.6)

PROJECT(DeformationFieldTransform)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(DeformationFieldTransform DeformationFieldTransform.cxx)
TARGET_LINK_LIBRARIES(DeformationFieldTransform ${ITK_LIBRARIES})
