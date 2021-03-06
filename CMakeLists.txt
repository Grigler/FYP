cmake_minimum_required(VERSION 2.6)
project(FYP)

include_directories(
  $(PROJECT_SOURCE_DIR)/src
  $(PROJECT_SOURCE_DIR)/include
)

if(WIN32)
  include_directories(
    ${PROJECT_SOURCE_DIR}/contrib/msvc/include
  )
  link_directories(
    ${PROJECT_SOURCE_DIR}/contrib/msvc/lib
  )
endif()

#----------------------------------------
# Building clew as a shared library
#----------------------------------------
  add_definitions(-DBUILD_SHARED_LIBRARY)

  add_library(clew SHARED
    src/clew/clew.c
  )
#----------------------------------------

file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR} src/FYP/*h)

#----------------------------------------
# Building FYP as a static library and
# linking to the clew shared library
#----------------------------------------
  add_library(FYP
    ${HEADERS}
  
    src/FYP/Context.cpp
    src/FYP/Pipeline.cpp
    src/FYP/Body.cpp
  )

  target_link_libraries(FYP clew)
#----------------------------------------


add_definitions(-DGLEW_STATIC)
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR} src/Demo/*h)
#----------------------------------------
# Building Demo as executable and linking
# to the FYP static library
#----------------------------------------
  add_executable(Demo
    ${HEADERS}
  
    src/demo/tiny_obj_loader.cc
  
    src/demo/RenderContext.cpp
  
    src/demo/main.cpp
  )

  target_link_libraries(Demo FYP glew32s)
#----------------------------------------

#----------------------------------------
# CPU implemenetation of simple RB-sys
#----------------------------------------
  add_executable(cpu
    src/cpu/main.cpp
	
	src/cpu/RB.cpp
  )
#----------------------------------------