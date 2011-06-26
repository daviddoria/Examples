cmake_minimum_required(VERSION 2.6)

PROJECT(Variables)

#string variable
#SET(VariableName DefaultValue CACHE VariableType HelpText FORCE)
SET(AStringVariable HelloWorld CACHE STRING "String variable help text" FORCE)

SET(ABoolVariable OFF CACHE BOOL "Bool varibale help text" FORCE)

ADD_EXECUTABLE(Variables Variables.cxx)

