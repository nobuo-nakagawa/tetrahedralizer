cmake_minimum_required(VERSION 3.6)

project(tetrahedralizer)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)
add_executable(tetrahedralizer ./src/tetrahedralizer.cpp
)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT tetrahedralizer)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_SUPPRESS_REGENERATION true)

if(MSVC)
add_definitions(-D_UNICODE -DUNICODE -D_CRT_SECURE_NO_WARNINGS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")

set(CompilerFlags
  CMAKE_CXX_FLAGS
  CMAKE_CXX_FLAGS_DEBUG
  CMAKE_CXX_FLAGS_RELEASE
  CMAKE_C_FLAGS
  CMAKE_C_FLAGS_DEBUG
  CMAKE_C_FLAGS_RELEASE
)
foreach(CompilerFlag ${CompilerFlags})
  string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

endif(MSVC)

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_definitions(-std=gnu++2a)
  set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")
#  set(CMAKE_OSX_ARCHITECTURES "x86_64")
endif()

if (CMAKE_SYSTEM_NAME MATCHES "Linux" )
  add_definitions(-std=c++2a)
endif()
