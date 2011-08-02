cmake_minimum_required(VERSION 2.6)

PROJECT(StatisticsOpeningLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(StatisticsOpeningLabelMapFilter StatisticsOpeningLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(StatisticsOpeningLabelMapFilter ${ITK_LIBRARIES} )
