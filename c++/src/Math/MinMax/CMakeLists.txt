cmake_minimum_required(VERSION 2.6)

PROJECT(MinMax)

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(MinMax MinMax.cpp )
