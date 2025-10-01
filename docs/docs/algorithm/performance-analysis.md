---
sidebar_position: 3
---

# Performance Analysis

This page presents benchmark results and performance characteristics of the Spanny3 RRT implementation, measured using Google Benchmark on Ubuntu Linux.

## Benchmark Configuration

All benchmarks use:
- **Compiler**: Clang 18 with libc++
- **Build Type**: Debug (optimizations disabled for development)
- **Platform**: Ubuntu Linux (GitHub Actions runner)
- **Fixed Seed**: 42 (for deterministic results)
- **Tool**: Google Benchmark v1.9.1

:::note
Debug builds are significantly slower than Release builds. For production use, compile with `-DCMAKE_BUILD_TYPE=Release` for 5-10x speedup.
:::

## End-to-End RRT Planning

### Simple Scenario (No Obstacles)

Open space from (-0.4, -0.4) to (0.4, 0.4) with no obstacles.

| Tree Size | Mean Time | Std Dev | Notes |
|-----------|-----------|---------|-------|
| 100 nodes | 0.18 ms  | ±0.01 ms | Fast convergence |
| 1000 nodes | 0.18 ms | ±0.01 ms | Reaches goal early |

**Observation**: Small tree size sufficient for simple scenes. Algorithm terminates upon reaching goal, so larger expansion limits don't add overhead.

### Complex Scenario (4 Obstacles)

Same start/goal with 4 circular obstacles (see `config/scenario.json`).

| Tree Size | Mean Time | Std Dev | Performance vs Simple |
|-----------|-----------|---------|----------------------|
| 100 nodes | 0.22 ms  | ±0.01 ms | +22% |
| 1000 nodes | 1.7 ms | ±0.1 ms | +850% |

**Observations**:
- Obstacles increase planning time by ~22% for small trees
- Large trees show significant slowdown (collision checking overhead)
- Most scenarios solvable with < 1000 nodes

### Success Rate

Success rate depends on:
- Obstacle density and placement
- Expansion limit
- Sample distance relative to obstacle gaps

No quantitative success rate data available yet (tracked in IMPROVEMENT_PLAN.md).

## Component Benchmarks

### Collision Detection Scaling

Line-circle intersection performance vs. obstacle count.

| Obstacles | Time per Check | Scaling Factor |
|-----------|---------------|----------------|
| 1         | 27 ns         | 1.0x (baseline) |
| 10        | 27 ns         | 1.0x |
| 100       | 27 ns         | 1.0x |

**Analysis**:
- **Constant time behavior** in practice
- O(m) complexity hidden by:
  - Small absolute times (nanoseconds)
  - Early termination on collision
  - CPU branch prediction
- Linear search adequate for &lt; 100 obstacles

**Recommendation**: For &gt; 1000 obstacles, consider spatial data structures (quadtree, BVH).

### Nearest Neighbor Search

Linear search performance vs. tree size.

| Tree Size | Time per Query | Scaling |
|-----------|---------------|---------|
| 10 nodes  | 0.23 μs      | Linear baseline |
| 100 nodes | 2.3 μs       | 10x (expected) |
| 1000 nodes | 20.2 μs     | 88x (~90x expected) |

**Analysis**:
- Clear **O(n) linear scaling**
- Becomes bottleneck for large trees (&gt; 1000 nodes)
- Each RRT iteration requires one nearest neighbor query

**Optimization Opportunity**:
- For trees with &gt; 10,000 nodes, implement **KD-tree**
- Expected improvement: O(n) → O(log n)
- Would reduce 1000-node query from 20μs to ~3μs

Current implementation prioritizes simplicity over scalability, which is appropriate for typical robotics scenarios (&lt; 1000 nodes).

## Complexity Summary

| Operation | Current | Optimal | When to Optimize |
|-----------|---------|---------|------------------|
| Nearest neighbor | O(n) | O(log n) with KD-tree | Trees &gt; 10,000 nodes |
| Collision check | O(m) | O(log m) with BVH | Obstacles &gt; 1000 |
| Tree expansion | O(n × m) | O(log n × log m) | Both conditions |

## Performance Bottlenecks

### Current Bottlenecks (by contribution)

1. **Nearest Neighbor Search** (dominant for large trees)
   - Linear search scales poorly
   - Each iteration requires full tree scan
   - 20μs per query for 1000 nodes

2. **Collision Detection** (dominant for many obstacles)
   - O(m) checks per edge
   - Minor overhead for typical scenes (&lt;100 obstacles)
   - 27ns per obstacle check

3. **Random Sampling** (negligible)
   - Uniform distribution generation
   - &lt; 1μs per sample
   - Not a bottleneck

### Optimization Priority

For typical use cases (&lt; 1000 nodes, &lt; 100 obstacles):
1. **No optimization needed** - current performance sufficient
2. Algorithm completes in &lt; 2ms for complex scenes

For large-scale scenarios (&gt; 10,000 nodes):
1. **High priority**: Implement KD-tree for nearest neighbor
2. **Medium priority**: Path extraction optimization
3. **Low priority**: Spatial hashing for collision detection

## Memory Usage

No quantitative measurements yet, but theoretical analysis:

| Component | Space Complexity | Typical Usage (1000 nodes) |
|-----------|-----------------|---------------------------|
| Nodes | O(n) | ~32 KB (32 bytes/node) |
| Edges | O(n) | ~32 KB (32 bytes/edge) |
| Obstacles | O(m) | ~2.4 KB (24 bytes × 100) |
| **Total** | **O(n + m)** | **~66 KB** |

Memory is not a concern for typical scenarios. Trees with millions of nodes would require memory optimization.

:::note
For large-scale planning with &gt; 10,000 nodes, consider implementing spatial data structures for improved performance.
:::

## Running Benchmarks

### Local Execution

```bash
# Run with console output
pixi run benchmark

# Save JSON results
pixi run benchmark-json

# View JSON results
cat build/benchmark_results.json
```

### CI Results

Benchmarks run automatically on every push to main:
1. Navigate to [GitHub Actions](https://github.com/griswaldbrooks/spanny3/actions)
2. Select latest workflow run
3. View "Benchmark Results" in step summary
4. Download JSON artifact for detailed analysis

### Interpreting Results

Benchmark output format:
```
Benchmark                           Time      CPU   Iterations
-----------------------------------------------------------------
BM_RRTPlanning_Simple/100        0.18 ms  0.18 ms     3947
BM_CollisionDetection/1          27 ns    27 ns       25683142
```

- **Time**: Wall clock time per iteration
- **CPU**: CPU time per iteration
- **Iterations**: Repetitions to achieve statistical significance

Google Benchmark automatically determines iteration count for reliable measurements.

## Comparison with Other Implementations

No direct comparisons available. General RRT performance expectations:

- **Simple scenes**: Sub-millisecond planning
- **Complex scenes**: 1-100ms depending on resolution
- **Very large trees**: Seconds without spatial data structures

Spanny3 performance is typical for educational RRT implementations. Production libraries (OMPL, MoveIt) use optimized data structures and achieve 10-100x speedups.

## Future Improvements

Tracked in IMPROVEMENT_PLAN.md:

1. **Performance benchmarks**: Add memory profiling
2. **Success rate analysis**: Measure vs. obstacle density
3. **Release build benchmarks**: Quantify optimization impact
4. **Regression detection**: Fail CI if performance degrades
5. **KD-tree implementation**: For large-scale scenarios

## See Also

- [RRT Overview](./rrt-overview.md) - Algorithm details
- [Collision Detection](./collision-detection.md) - Collision mathematics
- [Benchmarking Example](../examples/benchmarking.md) - Running custom benchmarks
- [GitHub Actions](https://github.com/griswaldbrooks/spanny3/actions) - Live CI results
