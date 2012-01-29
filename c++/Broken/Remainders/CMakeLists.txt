cmake_minimum_required(VERSION 2.6)

PROJECT(Remainders)

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(Remainders Remainders.cpp )
