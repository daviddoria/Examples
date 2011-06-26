cmake_minimum_required(VERSION 2.6)

PROJECT(LineNumberTest)

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(LineNumber LineNumber.cpp )
