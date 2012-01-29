cmake_minimum_required(VERSION 2.6)

Project(ThreadCommunication)

ADD_EXECUTABLE(ThreadCommunication ThreadCommunication.cpp)
TARGET_LINK_LIBRARIES(ThreadCommunication boost_thread-mt)
