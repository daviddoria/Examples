cmake_minimum_required(VERSION 2.6)

PROJECT(Read)

#required packages: libtiff4-devel libgeotiff-devel

include_directories(/usr/include/geotiff)

ADD_EXECUTABLE(Read Read.cpp )
