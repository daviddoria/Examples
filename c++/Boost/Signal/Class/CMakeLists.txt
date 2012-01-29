cmake_minimum_required(VERSION 2.6)

Project(SignalClass)

ADD_EXECUTABLE(SignalClass Class.cpp)
TARGET_LINK_LIBRARIES(SignalClass boost_signals-mt)
