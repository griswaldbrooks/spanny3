cmake_minimum_required(VERSION 3.22.1)
project(spanny CXX)

add_subdirectory(third_party SYSTEM)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

# Use Clang's standard library to get more of the C++23 features, std::print, std::mdspan
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")

add_library(json INTERFACE)
add_library(nlohmann::json ALIAS json)
target_include_directories(json INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/nlohmann>)
target_compile_features(json INTERFACE cxx_std_20)

add_library(cxxopts INTERFACE)
add_library(cxxopts::cxxopts ALIAS cxxopts)
target_include_directories(cxxopts INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/cxxopts>)
target_compile_features(cxxopts INTERFACE cxx_std_20)

add_executable(rrt
  src/rrt.cpp
)
target_link_libraries(rrt PRIVATE mdspan nlohmann::json cxxopts::cxxopts)
target_include_directories(rrt PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(rrt PRIVATE cxx_std_23)

if(NOT PROJECT_IS_TOP_LEVEL)
  return()
endif()
