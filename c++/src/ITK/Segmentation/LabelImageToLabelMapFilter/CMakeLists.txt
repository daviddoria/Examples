cmake_minimum_required(VERSION 2.6)

PROJECT(LabelImageToLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelImageToLabelMapFilter LabelImageToLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(LabelImageToLabelMapFilter ${ITK_LIBRARIES})
