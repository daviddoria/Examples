cmake_minimum_required(VERSION 2.6)

Project(BoostThreads)

find_package(Boost COMPONENTS thread REQUIRED)

ADD_EXECUTABLE(BoostThreads BoostThreads.cpp)
TARGET_LINK_LIBRARIES(BoostThreads ${Boost_THREAD_LIBRARY})
