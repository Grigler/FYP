cmake_minimum_required(VERSION 2.6)
project(FYP)

include_directories(
  $(PROJECT_SOURCE_DIR)/src
)

add_executable(fyp
  src/clew/clew.c  

  src/fyp/Context.cpp

  src/fyp/main.cpp
)