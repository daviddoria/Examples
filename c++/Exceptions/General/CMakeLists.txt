cmake_minimum_required(VERSION 2.6)

PROJECT(Exceptions)

ADD_EXECUTABLE(Exceptions Exceptions.cpp )

ADD_EXECUTABLE(ExceptionInFunction ExceptionInFunction.cpp )

ADD_EXECUTABLE(CustomException CustomException.cpp )

ADD_EXECUTABLE(runtime_error runtime_error.cpp )
