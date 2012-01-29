cmake_minimum_required(VERSION 2.6)

PROJECT(ginacExample)

ADD_EXECUTABLE(ginacExample ginacExample.cpp)

TARGET_LINK_LIBRARIES(ginacExample ginac cln)
