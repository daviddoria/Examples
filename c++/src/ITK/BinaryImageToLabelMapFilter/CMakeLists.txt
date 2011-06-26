cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryImageToLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryImageToLabelMapFilter BinaryImageToLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(BinaryImageToLabelMapFilter
${ITK_LIBRARIES} )
