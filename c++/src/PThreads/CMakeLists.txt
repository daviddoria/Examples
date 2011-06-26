cmake_minimum_required(VERSION 2.6)

Project(PThreads)

ADD_EXECUTABLE(PThreads PThreads.cpp)
TARGET_LINK_LIBRARIES(PThreads pthread)
