cmake_minimum_required(VERSION 2.6)

PROJECT(Thread)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(Thread Thread.cpp )
TARGET_LINK_LIBRARIES(Thread ${wxWidgets_LIBRARIES})
