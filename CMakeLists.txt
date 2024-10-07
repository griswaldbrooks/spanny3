cmake_minimum_required(VERSION 3.28.3)
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
target_compile_features(json INTERFACE cxx_std_23)

add_library(cxxopts INTERFACE)
add_library(cxxopts::cxxopts ALIAS cxxopts)
target_include_directories(cxxopts INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/cxxopts>)
target_compile_features(cxxopts INTERFACE cxx_std_23)

add_library(rrt SHARED
  src/rrt.cpp
  )
target_include_directories(rrt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(rrt PRIVATE cxx_std_23)

add_executable(rrt_cli
  src/rrt_cli.cpp
)
target_link_libraries(rrt_cli PRIVATE rrt nlohmann::json cxxopts::cxxopts)
target_compile_features(rrt_cli PRIVATE cxx_std_23)

if(NOT PROJECT_IS_TOP_LEVEL)
  return()
endif()
