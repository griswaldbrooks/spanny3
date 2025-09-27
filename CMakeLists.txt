cmake_minimum_required(VERSION 3.28.3)
project(spanny CXX)

add_subdirectory(third_party SYSTEM)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

# Use Clang's standard library to get more of the C++23 features, std::print, std::mdspan
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")

# Coverage configuration
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  message(STATUS "Building with coverage instrumentation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-instr-generate")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-instr-generate")

  # Create a list to track all project libraries for coverage
  set(PROJECT_LIBRARIES "" CACHE INTERNAL "List of project libraries for coverage")
endif()

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
add_library(spanny::rrt ALIAS rrt)
target_include_directories(rrt PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(rrt PRIVATE cxx_std_23)

# Add to project libraries list for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  set(PROJECT_LIBRARIES "${PROJECT_LIBRARIES};rrt" CACHE INTERNAL "List of project libraries for coverage")
endif()

add_executable(rrt_cli
  src/rrt_cli.cpp
)
target_link_libraries(rrt_cli PRIVATE spanny::rrt nlohmann::json cxxopts::cxxopts)
target_compile_features(rrt_cli PRIVATE cxx_std_23)

if(NOT PROJECT_IS_TOP_LEVEL)
  return()
endif()

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

option(BUILD_BENCHMARKS "Build benchmarks" ON)
if(BUILD_BENCHMARKS)
  include(FetchContent)
  FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.1
  )
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Disable benchmark testing")
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "Disable benchmark install")
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "Disable benchmark gtest tests")
  set(BENCHMARK_USE_BUNDLED_GTEST OFF CACHE BOOL "Use bundled gtest")
  FetchContent_MakeAvailable(benchmark)

  add_executable(rrt_benchmark
    benchmark/benchmark_rrt.cpp
  )
  target_link_libraries(rrt_benchmark PRIVATE spanny::rrt benchmark::benchmark)
  target_compile_features(rrt_benchmark PRIVATE cxx_std_23)
endif()

# Coverage targets
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  find_program(LLVM_COV_PATH NAMES llvm-cov-18 llvm-cov REQUIRED)
  find_program(LLVM_PROFDATA_PATH NAMES llvm-profdata-18 llvm-profdata REQUIRED)

  add_custom_target(coverage-clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${PROJECT_BINARY_DIR}/coverage
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/coverage/profiles
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/coverage/reports
    COMMENT "Cleaning coverage data"
  )

  add_custom_target(coverage-run
    COMMAND ${CMAKE_COMMAND} -E env LLVM_PROFILE_FILE=${PROJECT_BINARY_DIR}/coverage/profiles/test_%p.profraw
            ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS test_rrt
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMENT "Running tests with coverage collection"
  )

  add_custom_target(coverage-merge
    COMMAND ${LLVM_PROFDATA_PATH} merge -sparse
            ${PROJECT_BINARY_DIR}/coverage/profiles/*.profraw
            -o ${PROJECT_BINARY_DIR}/coverage/merged.profdata
    DEPENDS coverage-run
    COMMENT "Merging coverage profiles"
  )

  # Generate object file arguments for all project libraries
  set(COVERAGE_OBJECTS "")
  foreach(lib ${PROJECT_LIBRARIES})
    list(APPEND COVERAGE_OBJECTS -object $<TARGET_FILE:${lib}>)
  endforeach()

  add_custom_target(coverage-report
    COMMAND ${LLVM_COV_PATH} show $<TARGET_FILE:test_rrt>
            ${COVERAGE_OBJECTS}
            -instr-profile=${PROJECT_BINARY_DIR}/coverage/merged.profdata
            -format=html -output-dir=${PROJECT_BINARY_DIR}/coverage/reports/html
            -ignore-filename-regex '.*(third_party|test|_deps|build).*'
            ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include
    COMMAND ${LLVM_COV_PATH} report $<TARGET_FILE:test_rrt>
            ${COVERAGE_OBJECTS}
            -instr-profile=${PROJECT_BINARY_DIR}/coverage/merged.profdata
            -ignore-filename-regex '.*(third_party|test|_deps|build).*'
            ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include
            > ${PROJECT_BINARY_DIR}/coverage/reports/coverage.txt
    DEPENDS coverage-merge
    COMMENT "Generating coverage reports"
  )

  add_custom_target(coverage
    DEPENDS coverage-report
    COMMENT "Full coverage analysis pipeline"
  )
endif()
