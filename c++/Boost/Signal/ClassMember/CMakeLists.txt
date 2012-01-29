cmake_minimum_required(VERSION 2.6)

Project(SignalClassMember)

ADD_EXECUTABLE(SignalClassMember ClassMember.cpp)
TARGET_LINK_LIBRARIES(SignalClassMember boost_signals-mt)
