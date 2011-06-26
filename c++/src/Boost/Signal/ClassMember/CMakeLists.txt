cmake_minimum_required(VERSION 2.6)

Project(ClassMember)

ADD_EXECUTABLE(ClassMember ClassMember.cpp)
TARGET_LINK_LIBRARIES(ClassMember boost_signals-mt)
