cmake_minimum_required(VERSION 2.6)

PROJECT(ConnectedComponentFunctorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ConnectedComponentFunctorImageFilter ConnectedComponentFunctorImageFilter.cxx)
TARGET_LINK_LIBRARIES(ConnectedComponentFunctorImageFilter ${ITK_LIBRARIES})
