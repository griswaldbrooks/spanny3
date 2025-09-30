---
sidebar_position: 3
---

# Benchmarking

Learn how to measure and analyze the performance of RRT path planning using Google Benchmark.

## Running Benchmarks

### Quick Start

Run all benchmarks with default settings:

```bash
pixi run benchmark
```

Output shows timing statistics:

```
Run on (8 X 3000 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 8192 KiB (x1)
---------------------------------------------------------------------------
Benchmark                                 Time             CPU   Iterations
---------------------------------------------------------------------------
rrt_simple_scenario/100                5.23 ms         5.23 ms          134
rrt_simple_scenario/500               26.45 ms        26.44 ms           26
rrt_simple_scenario/1000              53.12 ms        53.10 ms           13
rrt_with_obstacles/100                 8.91 ms         8.91 ms           78
rrt_with_obstacles/500                45.23 ms        45.21 ms           15
rrt_with_obstacles/1000               91.34 ms        91.29 ms            8
collision_detection_line_circle/1      0.123 us        0.123 us      5689234
collision_detection_line_circle/8      0.934 us        0.933 us       749821
collision_detection_line_circle/64     7.456 us        7.453 us        93876
collision_detection_line_circle/100   11.678 us       11.672 us        59923
find_neighbor/10                       0.234 us        0.234 us      2991045
find_neighbor/64                       1.456 us        1.455 us       481234
find_neighbor/512                     11.678 us       11.672 us        59923
find_neighbor/1000                    22.891 us       22.883 us        30567
```

### Save JSON Results

Export detailed results for analysis:

```bash
pixi run benchmark-json
```

Results saved to `build/benchmark_results.json`:

```json
{
  "context": {
    "date": "2024-03-15T10:30:45-08:00",
    "host_name": "my-computer",
    "executable": "build/rrt_benchmark",
    "num_cpus": 8,
    "mhz_per_cpu": 3000
  },
  "benchmarks": [
    {
      "name": "rrt_simple_scenario/100",
      "run_name": "rrt_simple_scenario/100",
      "run_type": "iteration",
      "repetitions": 0,
      "threads": 1,
      "iterations": 134,
      "real_time": 5.23,
      "cpu_time": 5.23,
      "time_unit": "ms"
    }
  ]
}
```

## Understanding Benchmark Results

### Time Metrics

- **Time**: Wall-clock time (real-world elapsed time)
- **CPU**: CPU time (excludes I/O waits)
- **Iterations**: Number of times benchmark was run

Google Benchmark runs each benchmark multiple times to get stable measurements.

### Interpreting Results

**rrt_simple_scenario/100**: 5.23 ms
- Plans a path in open space
- Expansion limit of 100 nodes
- Baseline performance (no collision checking overhead)

**rrt_with_obstacles/100**: 8.91 ms
- Same parameters but with 4 obstacles
- ~70% slower due to collision checking
- Quantifies collision detection overhead

**collision_detection_line_circle/64**: 7.45 μs
- Check line-circle collision with 64 obstacles
- Linear scaling: 64x obstacles ≈ 64x time
- Critical hot path in RRT algorithm

**find_neighbor/1000**: 22.9 μs
- Find nearest neighbor in tree of 1000 nodes
- O(n) linear search implementation
- Dominates runtime for large trees

## Benchmark Suite Overview

### End-to-End Planning

#### rrt_simple_scenario

Measures planning in obstacle-free environment:

```cpp
static void rrt_simple_scenario(benchmark::State& state) {
  auto const start = spanny::position_t{0., 0.};
  auto const goal = spanny::position_t{5., 5.};

  spanny::planning_context_t context{
      .x_limits = {-10., 10.},
      .y_limits = {-10., 10.},
      .expansion_limit = static_cast<std::size_t>(state.range(0)),
      .sample_distance = 0.5,
      .goal_probability = 0.05,
      .obstacles = {}
  };

  for (auto _ : state) {
    auto random_generator = spanny::stochastic::random_context_t{42};
    auto rrt = spanny::stochastic::rrt_t{random_generator};
    auto result = rrt(start, goal, context);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(rrt_simple_scenario)->Arg(100)->Arg(500)->Arg(1000);
```

**Parameterized by**: Expansion limit (100, 500, 1000 nodes)

#### rrt_with_obstacles

Measures planning with obstacles:

```cpp
static void rrt_with_obstacles(benchmark::State& state) {
  auto const start = spanny::position_t{0., 0.};
  auto const goal = spanny::position_t{8., 8.};

  std::vector<spanny::circle_t> obstacles{
      {2., 2., 1.}, {5., 3., 1.5}, {3., 6., 1.}, {6., 6., 1.2}
  };

  spanny::planning_context_t context{
      .x_limits = {-10., 10.},
      .y_limits = {-10., 10.},
      .expansion_limit = static_cast<std::size_t>(state.range(0)),
      .sample_distance = 0.5,
      .goal_probability = 0.05,
      .obstacles = obstacles
  };

  for (auto _ : state) {
    auto random_generator = spanny::stochastic::random_context_t{42};
    auto rrt = spanny::stochastic::rrt_t{random_generator};
    auto result = rrt(start, goal, context);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(rrt_with_obstacles)->Arg(100)->Arg(500)->Arg(1000);
```

**Parameterized by**: Expansion limit (100, 500, 1000 nodes)

### Component Benchmarks

#### collision_detection_line_circle

Measures line-circle collision detection performance:

```cpp
static void collision_detection_line_circle(benchmark::State& state) {
  auto const p1 = spanny::position_t{0., 0.};
  auto const p2 = spanny::position_t{10., 10.};

  std::vector<spanny::circle_t> obstacles;
  for (int i = 0; i < state.range(0); ++i) {
    obstacles.push_back({static_cast<double>(i),
                        static_cast<double>(i),
                        0.5});
  }

  for (auto _ : state) {
    bool result = spanny::in_collision(p1, p2, obstacles);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(collision_detection_line_circle)->Range(1, 100);
```

**Parameterized by**: Number of obstacles (1, 8, 64, 100)

#### find_neighbor

Measures nearest neighbor search:

```cpp
static void find_neighbor(benchmark::State& state) {
  auto const search_node = spanny::node_t{spanny::position_t{5., 5.}};

  std::vector<spanny::node_t> nodes;
  for (int i = 0; i < state.range(0); ++i) {
    nodes.emplace_back(spanny::position_t{static_cast<double>(i),
                                          static_cast<double>(i)});
  }

  for (auto _ : state) {
    auto result = spanny::stochastic::find_neighbor(search_node, nodes);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(find_neighbor)->Range(10, 1000);
```

**Parameterized by**: Tree size (10, 64, 512, 1000 nodes)

## Creating Custom Benchmarks

### Basic Benchmark

Add to `benchmark/benchmark_rrt.cpp`:

```cpp
static void my_custom_benchmark(benchmark::State& state) {
  // Setup (outside timing loop)
  auto start = spanny::position_t{0., 0.};
  auto goal = spanny::position_t{10., 10.};

  // Benchmark loop
  for (auto _ : state) {
    // Code to measure
    auto result = my_function(start, goal);

    // Prevent optimization
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(my_custom_benchmark);
```

### Parameterized Benchmark

Test with different input sizes:

```cpp
static void parameterized_benchmark(benchmark::State& state) {
  int size = state.range(0);  // Get parameter

  // Setup with parameter
  std::vector<spanny::node_t> nodes;
  for (int i = 0; i < size; ++i) {
    nodes.emplace_back(spanny::position_t{
        static_cast<double>(i),
        static_cast<double>(i)
    });
  }

  for (auto _ : state) {
    auto result = process_nodes(nodes);
    benchmark::DoNotOptimize(result);
  }
}
// Test with 10, 100, 1000
BENCHMARK(parameterized_benchmark)->Arg(10)->Arg(100)->Arg(1000);

// Or use Range for powers of 2
BENCHMARK(parameterized_benchmark)->Range(8, 8192);
```

### Multiple Parameters

Test multiple dimensions:

```cpp
static void two_param_benchmark(benchmark::State& state) {
  int tree_size = state.range(0);
  int obstacle_count = state.range(1);

  // Setup
  std::vector<spanny::node_t> nodes;
  for (int i = 0; i < tree_size; ++i) {
    nodes.emplace_back(spanny::position_t{
        static_cast<double>(i), 0.
    });
  }

  std::vector<spanny::circle_t> obstacles;
  for (int i = 0; i < obstacle_count; ++i) {
    obstacles.push_back({static_cast<double>(i), 0., 0.5});
  }

  for (auto _ : state) {
    auto result = process(nodes, obstacles);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(two_param_benchmark)
    ->Args({100, 10})   // 100 nodes, 10 obstacles
    ->Args({100, 50})
    ->Args({1000, 10})
    ->Args({1000, 50});
```

## Advanced Benchmark Features

### Time Units

Specify appropriate time units:

```cpp
BENCHMARK(fast_function)->Unit(benchmark::kMicrosecond);
BENCHMARK(slow_function)->Unit(benchmark::kMillisecond);
BENCHMARK(very_slow_function)->Unit(benchmark::kSecond);
```

### Manual Timing

For more control over what's measured:

```cpp
static void manual_timing_benchmark(benchmark::State& state) {
  // Setup (not timed)
  auto data = create_large_dataset();

  for (auto _ : state) {
    state.PauseTiming();   // Stop timer
    auto input = prepare_input(data);
    state.ResumeTiming();  // Resume timer

    // Only this is timed
    auto result = expensive_computation(input);
    benchmark::DoNotOptimize(result);
  }
}
BENCHMARK(manual_timing_benchmark)->UseManualTime();
```

### Complexity Analysis

Measure algorithmic complexity:

```cpp
static void complexity_benchmark(benchmark::State& state) {
  int n = state.range(0);
  std::vector<int> data(n);

  for (auto _ : state) {
    auto result = algorithm(data);
    benchmark::DoNotOptimize(result);
  }

  state.SetComplexityN(n);
}
BENCHMARK(complexity_benchmark)
    ->RangeMultiplier(2)
    ->Range(8, 8192)
    ->Complexity(benchmark::oN);  // Expected O(n)
```

Google Benchmark reports how well the actual complexity matches expected.

## Analyzing Results

### Comparing Performance

Run benchmarks before and after changes:

```bash
# Baseline
pixi run benchmark-json
cp build/benchmark_results.json baseline.json

# Make changes to code
# ...

# New results
pixi run benchmark-json
cp build/benchmark_results.json new.json

# Compare (using external tool)
benchmark-compare baseline.json new.json
```

### Identifying Bottlenecks

Look for:
1. **Superlinear scaling**: O(n²) or worse complexity
2. **High variance**: Unstable performance
3. **Unexpected differences**: Large gaps between similar cases

Example analysis:

```
find_neighbor/10      0.234 us
find_neighbor/64      1.456 us  (64/10 = 6.4x size, 6.2x time - good)
find_neighbor/512    11.678 us  (512/64 = 8x size, 8x time - good)
find_neighbor/1000   22.891 us  (1000/512 ≈ 2x size, 2x time - good)
```

Linear scaling confirms O(n) complexity.

### Performance Goals

Target performance metrics:

| Operation | Tree Size / Obstacles | Target Time |
|-----------|----------------------|-------------|
| Simple planning | 100 nodes | < 10 ms |
| Simple planning | 1000 nodes | < 100 ms |
| With obstacles | 100 nodes | < 20 ms |
| Collision check | 100 obstacles | < 20 μs |
| Find neighbor | 1000 nodes | < 50 μs |

## Best Practices

1. **Use fixed seeds**: Deterministic results via `random_context_t{42}`
2. **Warm up CPU**: Benchmark runs warm-up iterations automatically
3. **Minimize variance**: Close other applications during benchmarking
4. **Representative workloads**: Test realistic scenarios
5. **DoNotOptimize**: Prevent compiler from optimizing away code
6. **Appropriate granularity**: Microseconds for hot paths, milliseconds for end-to-end
7. **Multiple parameters**: Test scalability across dimensions
8. **Document baselines**: Track performance over time

## Troubleshooting

### High Variance

**Problem**: Results vary significantly between runs

**Solutions**:
- Close background applications
- Run on dedicated hardware
- Increase `MinTime`:
  ```cpp
  BENCHMARK(my_benchmark)->MinTime(2.0);  // Run for 2 seconds
  ```

### Optimization Issues

**Problem**: Benchmark measures nothing (optimized away)

**Solutions**:
- Use `DoNotOptimize()`:
  ```cpp
  benchmark::DoNotOptimize(result);
  ```
- Use `ClobberMemory()`:
  ```cpp
  benchmark::ClobberMemory();  // Prevent memory optimizations
  ```

### Long Running Benchmarks

**Problem**: Benchmarks take too long

**Solutions**:
- Reduce parameter ranges
- Increase time unit (ms instead of μs)
- Limit iterations:
  ```cpp
  BENCHMARK(slow_benchmark)->Iterations(10);
  ```

## CI Integration

Benchmarks run automatically in GitHub Actions:

```yaml
- name: Run benchmarks
  run: pixi run benchmark

- name: Save benchmark results
  run: pixi run benchmark-json

- name: Upload results
  uses: actions/upload-artifact@v3
  with:
    name: benchmark-results
    path: build/benchmark_results.json
```

Results are available as CI artifacts.

## Next Steps

- [Basic Planning](basic-planning.md) - Understanding RRT basics
- [Custom Obstacles](custom-obstacles.md) - Creating test scenarios
- [Testing Guide](../development/testing-guide.md) - Writing tests
- [Google Benchmark Guide](https://github.com/google/benchmark/blob/main/docs/user_guide.md) - Official documentation
