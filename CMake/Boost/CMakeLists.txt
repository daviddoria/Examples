cmake_minimum_required(VERSION 2.6)

set(Boost_DEBUG 1)

FIND_PACKAGE(Boost 1.47 REQUIRED)
IF (BOOST_FOUND)
    INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

PROJECT(BoostProgram)

add_library(BoostProgram BoostProgram.cpp)

