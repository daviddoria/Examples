cmake_minimum_required(VERSION 2.6)

PROJECT(ChangeInformationImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ChangeInformationImageFilter ChangeInformationImageFilter.cxx)
TARGET_LINK_LIBRARIES(ChangeInformationImageFilter ${ITK_LIBRARIES})
