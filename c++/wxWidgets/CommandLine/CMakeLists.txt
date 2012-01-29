cmake_minimum_required(VERSION 2.6)

PROJECT(CommandLine)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(CommandLine CommandLine.cpp CommandLineOptions.cpp )
TARGET_LINK_LIBRARIES(CommandLine ${wxWidgets_LIBRARIES})
