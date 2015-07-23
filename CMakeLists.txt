cmake_minimum_required(VERSION 2.8.3)
project(ROSVeryfy_generator)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(
/home/gazben/Projects/R5-COP-Verification/libs

)



add_executable(
  ROSVeryfy_generator
  generator/Construct.cpp
  generator/Construct.h
  generator/Eval.h
  generator/Function.cpp
  generator/Function.h
  generator/Generator.h
  generator/main.cpp
)

