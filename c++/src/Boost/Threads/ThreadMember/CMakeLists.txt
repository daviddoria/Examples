cmake_minimum_required(VERSION 2.6)

Project(BoostThreadsMember)

ADD_EXECUTABLE(BoostThreadsMember BoostThreadsMember.cpp)
TARGET_LINK_LIBRARIES(BoostThreadsMember boost_thread-mt)