/**
 * @file benchmark_rrt.cpp
 * @brief Performance benchmarks for the RRT path planning algorithm and supporting functions
 *
 * This file contains Google Benchmark micro-benchmarks for measuring the performance
 * characteristics of the RRT (Rapidly-Exploring Random Tree) algorithm implementation.
 * The benchmarks cover end-to-end planning scenarios as well as individual algorithmic
 * components like collision detection and nearest neighbor search.
 *
 * @note All benchmarks use a fixed random seed (42) to ensure deterministic and
 * reproducible timing measurements across runs.
 */

#include "spanny/rrt.hpp"
#include <benchmark/benchmark.h>
#include <random>

/**
 * @brief Benchmark RRT planning performance in an obstacle-free environment
 *
 * Measures the wall-clock time required to plan a path from origin (0,0) to goal (5,5)
 * in a 20x20 unit planning space with no obstacles. This represents the best-case
 * performance scenario where collision checking overhead is minimal.
 *
 * @param state Google Benchmark state object providing iteration control and range parameters
 *
 * @details
 * - Planning space: [-10, 10] x [-10, 10]
 * - Start position: (0, 0)
 * - Goal position: (5, 5)
 * - Euclidean distance: ~7.07 units
 * - Sample distance: 0.5 units per expansion
 * - Goal sampling probability: 5%
 * - Random seed: 42 (deterministic)
 * - Parameterized by: expansion_limit (number of nodes to explore)
 *
 * @note The benchmark uses DoNotOptimize() to prevent the compiler from
 * eliminating the RRT computation during optimization.
 */
static void rrt_simple_scenario(benchmark::State& state) {
  auto const start = spanny::position_t{0., 0.};
  auto const goal = spanny::position_t{5., 5.};

  spanny::planning_context_t context{.x_limits = {-10., 10.},
                                     .y_limits = {-10., 10.},
                                     .expansion_limit = static_cast<std::size_t>(state.range(0)),
                                     .sample_distance = 0.5,
                                     .goal_probability = 0.05,
                                     .obstacles = {}};

  for (auto _ : state) {
    auto random_generator = spanny::stochastic::random_context_t{42};
    auto rrt = spanny::stochastic::rrt_t{random_generator};
    auto result = rrt(start, goal, context);
    benchmark::DoNotOptimize(result);
  }
}

/**
 * @brief Benchmark RRT planning performance with multiple circular obstacles
 *
 * Measures planning time in a cluttered environment with four circular obstacles
 * of varying sizes. This benchmark represents a realistic scenario where the planner
 * must navigate around obstacles, requiring significant collision checking overhead.
 *
 * @param state Google Benchmark state object providing iteration control and range parameters
 *
 * @details
 * - Planning space: [-10, 10] x [-10, 10]
 * - Start position: (0, 0)
 * - Goal position: (8, 8)
 * - Euclidean distance: ~11.31 units
 * - Sample distance: 0.5 units per expansion
 * - Goal sampling probability: 5%
 * - Random seed: 42 (deterministic)
 * - Obstacles:
 *   - Circle at (2, 2) with radius 1.0
 *   - Circle at (5, 3) with radius 1.5
 *   - Circle at (3, 6) with radius 1.0
 *   - Circle at (6, 6) with radius 1.2
 * - Parameterized by: expansion_limit (number of nodes to explore)
 *
 * @note Expected to be slower than rrt_simple_scenario due to collision checking.
 * The performance difference quantifies the overhead of obstacle avoidance.
 */
static void rrt_with_obstacles(benchmark::State& state) {
  auto const start = spanny::position_t{0., 0.};
  auto const goal = spanny::position_t{8., 8.};

  std::vector<spanny::circle_t> obstacles{{2., 2., 1.}, {5., 3., 1.5}, {3., 6., 1.}, {6., 6., 1.2}};

  spanny::planning_context_t context{.x_limits = {-10., 10.},
                                     .y_limits = {-10., 10.},
                                     .expansion_limit = static_cast<std::size_t>(state.range(0)),
                                     .sample_distance = 0.5,
                                     .goal_probability = 0.05,
                                     .obstacles = obstacles};

  for (auto _ : state) {
    auto random_generator = spanny::stochastic::random_context_t{42};
    auto rrt = spanny::stochastic::rrt_t{random_generator};
    auto result = rrt(start, goal, context);
    benchmark::DoNotOptimize(result);
  }
}

/**
 * @brief Benchmark line-circle collision detection with varying obstacle counts
 *
 * Measures the performance of the collision detection algorithm that checks whether
 * a line segment intersects with any circular obstacles. This is a critical hot-path
 * function called during tree expansion in the RRT algorithm.
 *
 * @param state Google Benchmark state object providing iteration control and range parameters
 *
 * @details
 * - Line segment: from (0, 0) to (10, 10)
 * - Line length: ~14.14 units (diagonal)
 * - Obstacles: arranged along the diagonal path at integer coordinates
 * - Each obstacle has radius: 0.5 units
 * - Parameterized by: number of obstacles (1 to 100)
 *
 * The benchmark creates obstacles at positions (i, i) for i in [0, N), forming
 * a line of obstacles along the diagonal. This worst-case scenario requires
 * checking all obstacles in the list.
 *
 * @note Time complexity analysis: Expected O(n) linear scaling with obstacle count,
 * as each obstacle requires constant-time intersection test using quadratic formula.
 *
 * @see spanny::in_collision() for the implementation details
 */
static void collision_detection_line_circle(benchmark::State& state) {
  auto const p1 = spanny::position_t{0., 0.};
  auto const p2 = spanny::position_t{10., 10.};

  std::vector<spanny::circle_t> obstacles;
  for (int i = 0; i < state.range(0); ++i) {
    obstacles.push_back({static_cast<double>(i), static_cast<double>(i), 0.5});
  }

  for (auto _ : state) {
    bool result = spanny::in_collision(p1, p2, obstacles);
    benchmark::DoNotOptimize(result);
  }
}

/**
 * @brief Benchmark nearest neighbor search with varying tree sizes
 *
 * Measures the performance of finding the closest node in the RRT tree to a
 * given sample point. This is a core operation called during each tree expansion
 * and typically dominates the runtime of naive RRT implementations.
 *
 * @param state Google Benchmark state object providing iteration control and range parameters
 *
 * @details
 * - Search target: node at position (5, 5)
 * - Tree nodes: arranged along diagonal from (0, 0) to (N-1, N-1)
 * - Distance metric: Euclidean distance in 2D space
 * - Parameterized by: number of nodes in tree (10 to 1000)
 *
 * The current implementation uses linear search (std::ranges::min_element),
 * resulting in O(n) time complexity per query. This benchmark helps quantify
 * the performance impact and motivates potential optimizations like KD-trees.
 *
 * @note Performance implications:
 * - For trees with 1000 nodes: ~20 microseconds per query
 * - For 10,000+ node trees: spatial data structures (e.g., KD-tree) recommended
 * - This is typically the primary bottleneck for large-scale planning problems
 *
 * @see spanny::stochastic::find_neighbor() for the implementation
 */
static void find_neighbor(benchmark::State& state) {
  auto const search_node = spanny::node_t{spanny::position_t{5., 5.}};

  std::vector<spanny::node_t> nodes;
  for (int i = 0; i < state.range(0); ++i) {
    nodes.emplace_back(spanny::position_t{static_cast<double>(i), static_cast<double>(i)});
  }

  for (auto _ : state) {
    auto result = spanny::stochastic::find_neighbor(search_node, nodes);
    benchmark::DoNotOptimize(result);
  }
}

/**
 * @brief Register RRT simple scenario benchmark with multiple expansion limits
 *
 * Tests performance with 100, 500, and 1000 node expansion limits to characterize
 * how planning time scales with tree size in obstacle-free environments.
 */
BENCHMARK(rrt_simple_scenario)->Arg(100)->Arg(500)->Arg(1000)->Unit(benchmark::kMillisecond);

/**
 * @brief Register RRT with obstacles benchmark with multiple expansion limits
 *
 * Tests performance with 100, 500, and 1000 node expansion limits to characterize
 * how planning time scales with tree size when collision checking is required.
 */
BENCHMARK(rrt_with_obstacles)->Arg(100)->Arg(500)->Arg(1000)->Unit(benchmark::kMillisecond);

/**
 * @brief Register collision detection benchmark with logarithmically-spaced obstacle counts
 *
 * Tests collision detection overhead with 1, 8, 64, and 100 obstacles using
 * Range() which generates powers-of-2 values between endpoints.
 */
BENCHMARK(collision_detection_line_circle)->Range(1, 100)->Unit(benchmark::kMicrosecond);

/**
 * @brief Register nearest neighbor benchmark with logarithmically-spaced tree sizes
 *
 * Tests neighbor search performance with 10, 64, 512, and 1000 nodes to
 * characterize the linear scaling behavior of the current implementation.
 */
BENCHMARK(find_neighbor)->Range(10, 1000)->Unit(benchmark::kMicrosecond);

/**
 * @brief Main entry point for benchmark executable
 *
 * This macro expands to a main() function that:
 * - Parses command-line flags (e.g., --benchmark_format, --benchmark_out)
 * - Runs all registered benchmarks
 * - Reports results in the specified format (console, JSON, CSV)
 * - Handles warm-up iterations and statistical analysis
 */
BENCHMARK_MAIN();
