cmake_minimum_required(VERSION 2.6)

Project(Signal)

ADD_EXECUTABLE(Signal Signal.cpp)
TARGET_LINK_LIBRARIES(Signal boost_signals-mt)
