cmake_minimum_required(VERSION 2.6)

PROJECT(ObjectByObjectLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ObjectByObjectLabelMapFilter ObjectByObjectLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(ObjectByObjectLabelMapFilter ${ITK_LIBRARIES} )
