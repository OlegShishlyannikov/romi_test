cmake_minimum_required(VERSION 3.6)

project(romi_test)

enable_language(CXX)

file(GLOB PROJECT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

set(PROJECT_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc" )
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++20)
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})

add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES})




