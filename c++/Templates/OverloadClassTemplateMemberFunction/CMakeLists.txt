Project(OverloadClassTemplateMemberFunction)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(OverloadClassTemplateMemberFunction OverloadClassTemplateMemberFunction.cpp)

ADD_EXECUTABLE(Deduction Deduction.cpp)

ADD_EXECUTABLE(Deduction2 Deduction2.cpp)

#ADD_EXECUTABLE(ClassDeduction ClassDeduction.cpp)
