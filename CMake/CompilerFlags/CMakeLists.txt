cmake_minimum_required(VERSION 2.6)

PROJECT(CompilerFlags)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(CompilerFlags CompilerFlags.cxx)

#works, but not best-practice
#set_target_properties(CompilerFlags PROPERTIES COMPILE_FLAGS "-DUNIX")

#works fine
#set_target_properties(CompilerFlags PROPERTIES COMPILE_DEFINITIONS "UNIX;DAVID;")

#also works fine
#set(myvariable "UNIX; DAVID;")
#message("myvariable: " ${myvariable})

#best practice
list(APPEND myvariable "UNIX")
list(APPEND myvariable "DAVID")
message("myvariable: ${myvariable}")
message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
SET_PROPERTY(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS ${wxWidgets_DEFINITIONS} )
set_target_properties(CompilerFlags PROPERTIES COMPILE_DEFINITIONS "${myvariable}")

