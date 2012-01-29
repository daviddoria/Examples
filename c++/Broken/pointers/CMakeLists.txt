cmake_minimum_required(VERSION 2.6)

PROJECT(PointerTest)

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(PointerTest PointerTest.cpp )
