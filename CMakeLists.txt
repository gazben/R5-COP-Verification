cmake_minimum_required(VERSION 2.8.3)
project(monitor_generator)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -lboost_log -pthread -DBOOST_LOG_DYN_LINK")

include_directories(
    /usr/include
)

find_package(
  Boost 1.58.0 COMPONENTS
    system
    filesystem
    program_options
    log
    thread
    REQUIRED
)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(
  monitor_generator
  generator/generator.cpp
  generator/generator.h
  generator/ast_node.cpp
  generator/ast_node.h
  generator/ast_optimizer.cpp
  generator/ast_optimizer.h
  generator/block_generator.cpp
  generator/block_generator.h
  generator/connection_normalform_generator.cpp
  generator/connection_normalform_generator.h
  generator/generator_framework.h
  generator/generator_framework.cpp
  generator/main.cpp

  generator/syntx/alternation.cc
  generator/syntx/alternation.h
  generator/syntx/ast_draw.h
  generator/syntx/base_rule.cc
  generator/syntx/base_rule.h
  generator/syntx/character.cc
  generator/syntx/character.h
  generator/syntx/concatenation.cc
  generator/syntx/concatenation.h
  generator/syntx/debug_action.h
  generator/syntx/eol.cc
  generator/syntx/eol.h
  generator/syntx/epsilon.h
  generator/syntx/identifier.cc
  generator/syntx/identifier.h
  generator/syntx/integer.cc
  generator/syntx/integer.h
  generator/syntx/keyword.cc
  generator/syntx/keyword.h
  generator/syntx/option.cc
  generator/syntx/option.h
  generator/syntx/parser.h
  generator/syntx/range.cc
  generator/syntx/range.h
  generator/syntx/real.cc
  generator/syntx/real.h
  generator/syntx/repetition.cc
  generator/syntx/repetition.h
  generator/syntx/repetition_or_epsilon.cc
  generator/syntx/repetition_or_epsilon.h
  generator/syntx/rule.cc
  generator/syntx/rule.h
  generator/syntx/search.cc
  generator/syntx/search.h
  generator/syntx/string.cc
  generator/syntx/string.h
  generator/syntx/substring.cc
  generator/syntx/substring.h
  generator/syntx/whitespace.cc
  generator/syntx/whitespace.h
  generator/syntx/whitespace_not_newline.cc
  generator/syntx/whitespace_not_newline.h
  generator/syntx/enum_set.h
  generator/syntx/language.h
  generator/syntx/ltl.h
  generator/syntx/ltl.cc
  )
target_link_libraries(
        monitor_generator
        ${Boost_LIBRARIES}
)




