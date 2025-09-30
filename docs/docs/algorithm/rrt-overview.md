---
sidebar_position: 1
---

# RRT Algorithm Overview

The **Rapidly-Exploring Random Tree (RRT)** is a sampling-based path planning algorithm designed for efficiently searching high-dimensional spaces. Spanny3 implements a 2D version suitable for robotics applications.

## How RRT Works

RRT builds a tree structure by randomly sampling points in the configuration space and connecting them to the nearest existing node, gradually exploring the space until a path to the goal is found.

### Algorithm Steps

1. **Initialize**: Start with a tree containing only the start position
2. **Sample**: Randomly generate a point in the configuration space (or occasionally sample the goal)
3. **Find Nearest**: Locate the nearest node in the existing tree
4. **Extend**: Project from the nearest node toward the sample by a fixed distance
5. **Check Collision**: Verify the new edge doesn't intersect obstacles
6. **Add to Tree**: If valid, add the new node and edge to the tree
7. **Check Goal**: If the new node reaches the goal, return success
8. **Repeat**: Continue until goal is reached or expansion limit exceeded

### Key Features

#### Probabilistic Completeness
RRT is **probabilistically complete** - given infinite time, it will find a solution if one exists. In practice, this means:
- Higher expansion limits increase success probability
- No guarantee of finding the optimal path
- May fail if expansion limit is too low

#### Goal Biasing
Instead of pure random sampling, RRT uses **goal biasing**:
- With probability `goal_probability`, sample the goal position
- Otherwise, sample randomly from the configuration space
- Typical values: 0.05 to 0.15 (5-15% goal sampling)

This significantly improves convergence speed toward the target.

#### Fixed-Step Extension
When extending toward a sample, RRT projects a fixed `sample_distance`:
- Prevents excessively long edges
- Enables finer resolution around obstacles
- Makes collision detection more reliable

## Implementation Details

### Core Algorithm

```cpp
// From include/spanny/rrt.hpp
template <like::some_random_generator random_generator_t>
struct rrt_t {
  [[nodiscard]] std::expected<tree_t, std::string> operator()(
      position_t const& start,
      position_t const& goal,
      planning_context_t const& context) {

    auto tree = tree_t{};
    tree.nodes.emplace_back(start);
    auto const goal_node = node_t{goal};

    while (tree.nodes.size() < context.expansion_limit) {
      // Sample space (with goal biasing)
      auto const node_maybe = sample_space_or_goal(
          random_generator_, context, goal_node);

      // Expand tree toward sample
      auto const id_maybe = node_maybe.and_then(
          [&](auto const& node) {
            return expand_tree(context, node, tree);
          });

      // Check if we reached the goal
      if (id_maybe == goal_node.id) {
        return tree;
      }
    }

    return std::unexpected("RRT failed to reach goal");
  }
};
```

### Tree Expansion

The `expand_tree` function (src/rrt.cpp:107-121) performs the core extension logic:

1. **Find nearest neighbor** using linear search O(n)
2. **Project sample** to respect `sample_distance` limit
3. **Check collision** between nearest node and projected sample
4. **Add to tree** if collision-free

```cpp
std::expected<node_id_t, std::string> expand_tree(
    planning_context_t const& context,
    node_t sampled_node,
    tree_t& tree) {

  return find_neighbor(sampled_node, tree.nodes)
      .and_then([&](auto const& closest) {
        auto const sample = project_sample(context, sampled_node, closest);

        if (in_collision(closest.position, sample.position, context.obstacles)) {
          return std::unexpected("Sampled node was in collision");
        }

        auto const cost = heuristic(closest, sample);
        tree.nodes.emplace_back(sample.position);
        tree.edges.emplace_back(closest.id, sample.id, cost);

        return sample.id;
      });
}
```

## Configuration Parameters

The algorithm behavior is controlled by `planning_context_t`:

| Parameter | Type | Description | Typical Range |
|-----------|------|-------------|---------------|
| `x_limits` | `bounds_t` | Min/max x coordinates | Scene-dependent |
| `y_limits` | `bounds_t` | Min/max y coordinates | Scene-dependent |
| `expansion_limit` | `size_t` | Maximum tree nodes | 100-10,000 |
| `sample_distance` | `double` | Fixed extension distance | 0.05-0.2 |
| `goal_probability` | `double` | Goal sampling probability | 0.05-0.15 |
| `obstacles` | `vector<circle_t>` | Circular obstacles | N/A |

### Parameter Tuning Guidelines

**Expansion Limit**:
- Too low: May fail to find path
- Too high: Wastes computation
- Start with 1000 for simple scenes

**Sample Distance**:
- Smaller: Finer resolution, slower exploration
- Larger: Faster exploration, may miss narrow passages
- Balance based on obstacle density

**Goal Probability**:
- Higher: Faster convergence if straight path exists
- Lower: Better exploration in cluttered environments
- 0.1 (10%) is a good default

## Performance Characteristics

### Time Complexity
- **Nearest Neighbor**: O(n) per iteration (linear search)
- **Collision Check**: O(m) where m = number of obstacles
- **Overall**: O(n² × m) for n expansions

For large trees (>10,000 nodes), consider KD-tree for O(log n) nearest neighbor queries.

### Space Complexity
- **Nodes**: O(n) where n = expansion_limit
- **Edges**: O(n - 1) ≈ O(n)
- **Total**: O(n)

### Success Rate
Success probability depends on:
- Scene complexity (obstacle density)
- Expansion limit
- Sample distance relative to obstacle spacing

See [Performance Analysis](./performance-analysis.md) for benchmark results.

## Limitations

### Not Optimal
RRT finds **a** path, not the **best** path. Solutions are typically:
- Longer than optimal
- Have unnecessary turns
- Vary between runs due to randomness

For optimal paths, consider:
- **RRT\*** (RRT-star): Rewires tree for optimality
- **Informed RRT\***: Uses heuristics to focus search

### Local Minimum Susceptibility
In complex environments with narrow passages:
- May struggle to find solution
- Increasing `expansion_limit` helps
- Adjust `sample_distance` for narrow passages

### No Path Extraction (Current Implementation)
The current implementation (as of commit bc62bcd) **builds the tree** but does not extract the path. This is a known limitation tracked in IMPROVEMENT_PLAN.md.

## See Also

- [Collision Detection](./collision-detection.md) - Line-circle intersection mathematics
- [Performance Analysis](./performance-analysis.md) - Benchmark results and scaling
- [Planning Context API](../api/planning-context.md) - Configuration reference
- [RRT Planner API](../api/rrt-planner.md) - Using the planner in code
- [Doxygen Reference](/doxygen/html/) - Complete API documentation

## References

- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees: A New Tool for Path Planning*. Technical Report TR 98-11, Iowa State University.
- LaValle, S. M., & Kuffner, J. J. (2001). *Randomized Kinodynamic Planning*. International Journal of Robotics Research.
