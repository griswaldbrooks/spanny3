---
sidebar_position: 1
---

# Basic Path Planning

Learn the fundamentals of using the RRT planner with a minimal example.

## Minimal Example

Here's a complete program that plans a path from start to goal:

```cpp
#include "spanny/rrt.hpp"
#include <iostream>
#include <random>

int main() {
  // Define start and goal positions
  auto start = spanny::position_t{0.0, 0.0};
  auto goal = spanny::position_t{5.0, 5.0};

  // Configure planning parameters
  spanny::planning_context_t context{
      .x_limits = {-10.0, 10.0},    // Planning bounds: -10 to 10 in x
      .y_limits = {-10.0, 10.0},    // Planning bounds: -10 to 10 in y
      .expansion_limit = 1000,       // Maximum number of nodes
      .sample_distance = 0.5,        // Step size for tree expansion
      .goal_probability = 0.05,      // 5% chance of sampling goal
      .obstacles = {}                // No obstacles
  };

  // Create random generator and planner
  auto random_generator = spanny::stochastic::random_context_t{
      std::random_device{}()
  };
  auto planner = spanny::stochastic::rrt_t{random_generator};

  // Plan the path
  auto result = planner(start, goal, context);

  // Check result and print tree
  if (result.has_value()) {
    auto const& tree = result.value();
    std::cout << "Success! Tree has " << tree.nodes.size()
              << " nodes\n";

    // Print all nodes
    for (auto const& node : tree.nodes) {
      std::cout << "Node: (" << node.position.x << ", "
                << node.position.y << ")\n";
    }
  } else {
    std::cerr << "Planning failed: " << result.error() << "\n";
    return 1;
  }

  return 0;
}
```

## Building and Running

### Using Pixi

1. Add your file to `src/my_planner.cpp`

2. Update `CMakeLists.txt`:
   ```cmake
   add_executable(my_planner src/my_planner.cpp)
   target_link_libraries(my_planner PRIVATE spanny_lib)
   ```

3. Build and run:
   ```bash
   pixi run build
   ./build/my_planner
   ```

### Expected Output

```
Success! Tree has 247 nodes
Node: (0, 0)
Node: (0.447214, 0.447214)
Node: (0.894427, 0.894427)
...
Node: (4.89443, 4.89443)
Node: (5, 5)
```

## Understanding the Code

### Position Definition

```cpp
auto start = spanny::position_t{0.0, 0.0};
auto goal = spanny::position_t{5.0, 5.0};
```

`position_t` represents a 2D point with `x` and `y` coordinates.

### Planning Context

```cpp
spanny::planning_context_t context{
    .x_limits = {-10.0, 10.0},
    .y_limits = {-10.0, 10.0},
    .expansion_limit = 1000,
    .sample_distance = 0.5,
    .goal_probability = 0.05,
    .obstacles = {}
};
```

Configuration parameters:

- **x_limits, y_limits**: Boundaries of the planning space
- **expansion_limit**: Stop after this many nodes (prevents infinite loops)
- **sample_distance**: Fixed step size when extending tree toward samples
- **goal_probability**: Likelihood of sampling goal instead of random point
- **obstacles**: List of circular obstacles (empty for this example)

### Random Generator

```cpp
auto random_generator = spanny::stochastic::random_context_t{
    std::random_device{}()
};
```

RRT requires randomness for sampling. The `random_context_t` wraps `std::mt19937` with a seed from `std::random_device`.

For reproducible results, use a fixed seed:
```cpp
auto random_generator = spanny::stochastic::random_context_t{42};
```

### Creating the Planner

```cpp
auto planner = spanny::stochastic::rrt_t{random_generator};
```

The `rrt_t` template takes any object satisfying the `some_random_generator` concept:
- `double real_between(double min, double max)` - Random real in range
- `bool yes_maybe(double probability)` - Bernoulli trial

### Planning

```cpp
auto result = planner(start, goal, context);
```

The planner is callable and returns `std::expected<tree_t, std::string>`:
- **Success**: Contains `tree_t` with nodes and edges
- **Failure**: Contains error message string

### Handling Results

```cpp
if (result.has_value()) {
  auto const& tree = result.value();
  // Use tree
} else {
  std::cerr << result.error() << "\n";
}
```

Always check `has_value()` before accessing the tree.

Use modern C++23 style:
```cpp
result.transform([](auto const& tree) {
  std::cout << "Success with " << tree.nodes.size() << " nodes\n";
  return 0;
}).transform_error([](auto const& error) {
  std::cerr << "Failed: " << error << "\n";
  return 1;
});
```

## Common Failure Modes

### Planning Fails to Reach Goal

```
Planning failed: RRT failed to reach goal
```

**Causes:**
- `expansion_limit` too low
- `sample_distance` too small
- Goal is unreachable due to obstacles
- Bad luck with random sampling

**Solutions:**
- Increase `expansion_limit`:
  ```cpp
  .expansion_limit = 5000,
  ```
- Increase `goal_probability`:
  ```cpp
  .goal_probability = 0.1,  // 10% instead of 5%
  ```
- Increase `sample_distance`:
  ```cpp
  .sample_distance = 1.0,  // Larger steps
  ```

### Sampled Node in Collision

Internal error when sampling lands inside an obstacle. The planner retries automatically, but this can slow planning.

**Solution:** Adjust parameters to reduce collision likelihood:
```cpp
.sample_distance = 0.3,  // Smaller steps avoid obstacles better
```

## Interpreting Results

### The Tree Structure

The result is a `tree_t` with:

```cpp
struct tree_t {
  std::vector<node_t> nodes;  // All sampled points
  std::vector<edge_t> edges;  // Parent-child connections
};
```

### Nodes

```cpp
for (auto const& node : tree.nodes) {
  std::cout << "Node " << node.id << ": ("
            << node.position.x << ", " << node.position.y << ")\n";
}
```

Each node has:
- **id**: Unique identifier (hash of position)
- **position**: 2D location in space

### Edges

```cpp
for (auto const& edge : tree.edges) {
  std::cout << "Edge: " << edge.parent << " -> " << edge.child
            << " (cost: " << edge.cost << ")\n";
}
```

Each edge has:
- **parent**: Parent node ID
- **child**: Child node ID
- **cost**: Distance between parent and child

### Finding the Path

The current implementation doesn't extract the path from tree to goal. You'll need to traverse edges backwards from goal to start:

```cpp
std::vector<spanny::node_t> extract_path(
    spanny::tree_t const& tree,
    spanny::node_id_t goal_id) {

  std::vector<spanny::node_t> path;

  // Find goal node
  auto goal_it = std::ranges::find(tree.nodes, goal_id, &spanny::node_t::id);
  if (goal_it == tree.nodes.end()) {
    return {};  // Goal not in tree
  }

  // Trace back to start
  auto current_id = goal_id;
  while (true) {
    // Find node with current_id
    auto node_it = std::ranges::find(tree.nodes, current_id,
                                     &spanny::node_t::id);
    path.push_back(*node_it);

    // Find edge with current_id as child
    auto edge_it = std::ranges::find(tree.edges, current_id,
                                     &spanny::edge_t::child);
    if (edge_it == tree.edges.end()) {
      break;  // Reached start (no parent)
    }

    current_id = edge_it->parent;
  }

  std::ranges::reverse(path);  // Start to goal order
  return path;
}
```

## Tuning Parameters

### Exploration vs Exploitation

**High goal_probability** (e.g., 0.5):
- Faster convergence
- Less exploration
- May miss better paths

**Low goal_probability** (e.g., 0.01):
- More exploration
- Slower convergence
- Better path quality

### Step Size

**Large sample_distance** (e.g., 2.0):
- Fewer nodes needed
- Faster planning
- Coarser paths
- May miss narrow passages

**Small sample_distance** (e.g., 0.1):
- More nodes needed
- Slower planning
- Smoother paths
- Handles tight spaces

### Search Space

Tight bounds speed up planning:
```cpp
// Loose bounds (slow)
.x_limits = {-100.0, 100.0},
.y_limits = {-100.0, 100.0},

// Tight bounds (fast)
.x_limits = {-1.0, 6.0},   // Just around start and goal
.y_limits = {-1.0, 6.0},
```

## Next Steps

- [Custom Obstacles](custom-obstacles.md) - Add obstacles to your scenario
- [Benchmarking](benchmarking.md) - Measure planning performance
- [API: RRT Planner](../api/rrt-planner.md) - Detailed API reference
- [API: Planning Context](../api/planning-context.md) - Parameter tuning guide
