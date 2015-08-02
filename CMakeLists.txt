cmake_minimum_required(VERSION 2.8.3)
project(ROSVeryfy_generator)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -lboost_system")

include_directories(
 libs/
 /usr/include
)

find_package(Boost 1.58.0 COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(
  ROSVeryfy_generator
  generator/Construct.cpp
  generator/Construct.h
  generator/Eval.h
  generator/Function.cpp
  generator/Function.h
  generator/Generator.h
  generator/main.cpp

  libs/SyntX/util/parser/alternation.cc
  libs/SyntX/util/parser/alternation.h
  libs/SyntX/util/parser/ast_draw.cc
  libs/SyntX/util/parser/ast_draw.h
  libs/SyntX/util/parser/base_rule.cc
  libs/SyntX/util/parser/base_rule.h
  libs/SyntX/util/parser/character.cc
  libs/SyntX/util/parser/character.h
  libs/SyntX/util/parser/concatenation.cc
  libs/SyntX/util/parser/concatenation.h
  libs/SyntX/util/parser/debug_action.h
  libs/SyntX/util/parser/eol.cc
  libs/SyntX/util/parser/eol.h
  libs/SyntX/util/parser/epsilon.h
  libs/SyntX/util/parser/identifier.cc
  libs/SyntX/util/parser/identifier.h
  libs/SyntX/util/parser/integer.cc
  libs/SyntX/util/parser/integer.h
  libs/SyntX/util/parser/keyword.cc
  libs/SyntX/util/parser/keyword.h
  libs/SyntX/util/parser/option.cc
  libs/SyntX/util/parser/option.h
  libs/SyntX/util/parser/parser.h
  libs/SyntX/util/parser/range.cc
  libs/SyntX/util/parser/range.h
  libs/SyntX/util/parser/real.cc
  libs/SyntX/util/parser/real.h
  libs/SyntX/util/parser/repetition.cc
  libs/SyntX/util/parser/repetition.h
  libs/SyntX/util/parser/repetition_or_epsilon.cc
  libs/SyntX/util/parser/repetition_or_epsilon.h
  libs/SyntX/util/parser/rule.cc
  libs/SyntX/util/parser/rule.h
  libs/SyntX/util/parser/search.cc
  libs/SyntX/util/parser/search.h
  libs/SyntX/util/parser/string.cc
  libs/SyntX/util/parser/string.h
  libs/SyntX/util/parser/substring.cc
  libs/SyntX/util/parser/substring.h
  libs/SyntX/util/parser/whitespace.cc
  libs/SyntX/util/parser/whitespace.h
  libs/SyntX/util/parser/whitespace_not_newline.cc
  libs/SyntX/util/parser/whitespace_not_newline.h
  libs/SyntX/util/enum_set.h
  libs/SyntX/util/languages/language.h
  libs/SyntX/util/languages/ltl.h
  libs/SyntX/util/languages/ltl.cc
)

target_link_libraries(ROSVeryfy_generator ${Boost_LIBRARIES})


