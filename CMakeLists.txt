cmake_minimum_required(VERSION 2.6)
project(FYP)

include_directories(
  $(PROJECT_SOURCE_DIR)/src
  $(PROJECT_SOURCE_DIR)/include
)

#----------------------------------------
# Building clew as a shared library
#----------------------------------------
  add_definitions(-DBUILD_SHARED_LIBRARY)

  add_library(clew SHARED
    src/clew/clew.c
  )
#----------------------------------------


#----------------------------------------
# Building FYP as a static library and
# linking to the clew shared library
#----------------------------------------
  add_library(FYP
    src/fyp/Context.cpp
  
  )

  target_link_libraries(FYP clew)
#----------------------------------------


#----------------------------------------
# Building Demo as executable and linking
# to the FYP static library
#----------------------------------------
  add_executable(Demo
  
    src/demo/main.cpp
  )

  target_link_libraries(Demo FYP)
#----------------------------------------

#----------------------------------------
# CPU implemenetation of simple RB-sys
#----------------------------------------
  add_executable(cpu
    src/cpu/main.cpp
	
	src/cpu/RB.cpp
  )
#----------------------------------------