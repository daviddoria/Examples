cmake_minimum_required(VERSION 2.6)

PROJECT(KeypressObserver)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KeypressObserver KeypressObserver.cxx)
TARGET_LINK_LIBRARIES(KeypressObserver vtkHybrid)
