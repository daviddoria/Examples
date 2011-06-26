cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGM)

SET(OpenGM_IncludePath "~/src/opengm10/include" CACHE STRING "Path to OpenGM include/ directory")
# I set this to /home/doriad/src/opengm10/include

include_directories(${OpenGM_IncludePath})

FILE(GLOB headers ${OpenGM_IncludePath}/opengm/* ${OpenGM_IncludePath}/opengm/inference/*)

add_executable(OpenGM opengm.cpp ${headers})

add_executable(Big big.cpp)

add_executable(BigBP BigBP.cpp)

add_executable(EvaluateEnergy EvaluateEnergy.cpp)

add_executable(CustomVisitor CustomVisitor.cpp)