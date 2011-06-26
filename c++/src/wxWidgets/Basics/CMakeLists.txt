cmake_minimum_required(VERSION 2.6)

PROJECT(Basics)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(Basics Basics.cpp )
TARGET_LINK_LIBRARIES(Basics ${wxWidgets_LIBRARIES})
