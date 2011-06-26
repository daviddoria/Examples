cmake_minimum_required(VERSION 2.6)

Project(Class)

ADD_EXECUTABLE(Class Class.cpp)
TARGET_LINK_LIBRARIES(Class boost_signals-mt)
