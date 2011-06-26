cmake_minimum_required(VERSION 2.6)

Project(ThreadedInteractor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ThreadedInteractor ThreadedInteractor.cpp)
TARGET_LINK_LIBRARIES(ThreadedInteractor boost_thread-mt vtkHybrid)
