cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryImageToShapeLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryImageToShapeLabelMapFilter BinaryImageToShapeLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(BinaryImageToShapeLabelMapFilter
${ITK_LIBRARIES} )
