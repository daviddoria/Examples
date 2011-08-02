cmake_minimum_required(VERSION 2.6)

PROJECT(LabelImageToShapeLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelImageToShapeLabelMapFilter LabelImageToShapeLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(LabelImageToShapeLabelMapFilter ${ITK_LIBRARIES})
