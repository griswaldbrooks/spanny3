---
sidebar_position: 3
---

# RRT Planner

The `rrt_t` template class implements the Rapidly-Exploring Random Tree path planning algorithm.

## Overview

```cpp
template <like::some_random_generator random_generator_t>
struct rrt_t {
  explicit rrt_t(random_generator_t& random_generator);

  [[nodiscard]] std::expected<tree_t, std::string>
  operator()(position_t const& start,
             position_t const& goal,
             planning_context_t const& context);
};
```

## Template Parameters

### some_random_generator

The planner is generic over random number generators satisfying the `some_random_generator` concept:

```cpp
template <typename T>
concept some_random_generator = requires(T gen, double min, double max, double prob) {
  { gen.real_between(min, max) } -> std::convertible_to<double>;
  { gen.yes_maybe(prob) } -> std::convertible_to<bool>;
};
```

**Required Methods**:

```cpp
// Generate random real number in range [min, max]
double real_between(double min, double max);

// Perform Bernoulli trial with given probability
bool yes_maybe(double probability);
```

**Built-in Implementation**:

```cpp
struct random_context_t {
  explicit random_context_t(uint32_t seed);
  double real_between(double min, double max);
  bool yes_maybe(double probability);
};
```

Wraps `std::mt19937` for production use.

**Example Usage**:

```cpp
// Random generator with seed
auto rng = spanny::stochastic::random_context_t{42};

// Create planner with generator
auto planner = spanny::stochastic::rrt_t{rng};
```

## Constructor

```cpp
explicit rrt_t(random_generator_t& random_generator);
```

Creates an RRT planner that uses the provided random generator.

**Parameters**:
- `random_generator`: Reference to object satisfying `some_random_generator` concept

**Example**:

```cpp
auto rng = spanny::stochastic::random_context_t{
    std::random_device{}()  // Random seed
};

auto planner = spanny::stochastic::rrt_t{rng};
```

**Important**: The generator is stored by reference, so it must outlive the planner.

```cpp
// WRONG - generator goes out of scope
spanny::stochastic::rrt_t create_planner() {
  auto rng = spanny::stochastic::random_context_t{42};
  return spanny::stochastic::rrt_t{rng};  // Dangling reference!
}

// CORRECT - generator outlives planner
auto rng = spanny::stochastic::random_context_t{42};
auto planner = spanny::stochastic::rrt_t{rng};
```

## Planning Function

```cpp
[[nodiscard]] std::expected<tree_t, std::string>
operator()(position_t const& start,
           position_t const& goal,
           planning_context_t const& context);
```

Plans a path from start to goal using RRT algorithm.

**Parameters**:
- `start`: Starting position (must be collision-free)
- `goal`: Goal position (must be collision-free)
- `context`: Planning parameters and obstacles

**Returns**: `std::expected<tree_t, std::string>`
- **Success**: `tree_t` containing nodes and edges
- **Failure**: Error message string

**Example**:

```cpp
auto start = spanny::position_t{0.0, 0.0};
auto goal = spanny::position_t{5.0, 5.0};

spanny::planning_context_t context{
    .x_limits = {-10.0, 10.0},
    .y_limits = {-10.0, 10.0},
    .expansion_limit = 1000,
    .sample_distance = 0.5,
    .goal_probability = 0.05,
    .obstacles = {}
};

auto result = planner(start, goal, context);

if (result.has_value()) {
  auto const& tree = result.value();
  // Use tree
} else {
  std::cerr << "Planning failed: " << result.error() << "\n";
}
```

## Algorithm Details

### RRT Algorithm Steps

1. **Initialize**: Create tree with start node
2. **Sample**: Generate random position (or goal with probability)
3. **Nearest**: Find closest node in tree
4. **Extend**: Project from nearest toward sample by `sample_distance`
5. **Collision Check**: Verify edge is collision-free
6. **Add Node**: Insert new node and edge into tree
7. **Check Goal**: If new node is goal, return success
8. **Repeat**: Continue until goal reached or `expansion_limit` exceeded

### Pseudocode

```
function RRT(start, goal, context):
  tree = Tree(start)

  for i in 0 to context.expansion_limit:
    # Sample space
    if random() < context.goal_probability:
      sample = goal
    else:
      sample = random_position(context.x_limits, context.y_limits)

    # Find nearest node in tree
    nearest = find_closest(tree.nodes, sample)

    # Project toward sample
    new_node = project(nearest, sample, context.sample_distance)

    # Check collision
    if in_collision(nearest, new_node, context.obstacles):
      continue

    # Add to tree
    tree.add_node(new_node)
    tree.add_edge(nearest, new_node)

    # Check if goal reached
    if new_node == goal:
      return tree

  return error("RRT failed to reach goal")
```

### Key Properties

**Probabilistically Complete**: Given infinite time, RRT will find a path if one exists

**Non-Optimal**: Path quality depends on randomness; not guaranteed to find shortest path

**Fast**: Explores space quickly with random sampling

**Single-Query**: Builds tree from scratch for each query (no preprocessing)

## Return Type Handling

### Using std::expected

The result uses C++23 `std::expected` for error handling without exceptions.

**Basic Handling**:

```cpp
auto result = planner(start, goal, context);

if (result.has_value()) {
  // Success path
  auto const& tree = result.value();
  process_tree(tree);
} else {
  // Error path
  std::cerr << "Error: " << result.error() << "\n";
}
```

**Using transform**:

```cpp
auto result = planner(start, goal, context)
    .transform([](auto const& tree) {
      std::cout << "Success! " << tree.nodes.size() << " nodes\n";
      return 0;  // Return success code
    })
    .transform_error([](auto const& error) {
      std::cerr << "Failed: " << error << "\n";
      return 1;  // Return error code
    });

int exit_code = result.value_or(result.error());
```

**Using and_then**:

```cpp
auto path = planner(start, goal, context)
    .and_then([](auto const& tree) {
      return extract_path(tree);  // Returns std::expected
    })
    .transform([](auto const& path) {
      smooth_path(path);
      return path;
    });

if (path.has_value()) {
  visualize_path(path.value());
}
```

**Error Messages**:

Possible error strings:
- `"RRT failed to reach goal"` - Expansion limit reached
- `"Sampled node is in collision with obstacle"` - (Internal, retried automatically)

## Usage Patterns

### Basic Usage

```cpp
#include "spanny/rrt.hpp"
#include <random>

int main() {
  // Setup
  auto start = spanny::position_t{0.0, 0.0};
  auto goal = spanny::position_t{5.0, 5.0};

  spanny::planning_context_t context{
      .x_limits = {-10.0, 10.0},
      .y_limits = {-10.0, 10.0},
      .expansion_limit = 1000,
      .sample_distance = 0.5,
      .goal_probability = 0.05,
      .obstacles = {}
  };

  // Plan
  auto rng = spanny::stochastic::random_context_t{
      std::random_device{}()
  };
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // Handle result
  if (!result.has_value()) {
    return 1;
  }

  auto const& tree = result.value();
  std::cout << "Success with " << tree.nodes.size() << " nodes\n";

  return 0;
}
```

### Reusable Planner

```cpp
// Create once, use many times
auto rng = spanny::stochastic::random_context_t{42};
auto planner = spanny::stochastic::rrt_t{rng};

// Plan multiple paths
for (auto const& [start, goal] : queries) {
  auto result = planner(start, goal, context);
  if (result.has_value()) {
    process_tree(result.value());
  }
}
```

### Reproducible Planning

Use fixed seed for deterministic results:

```cpp
// Same seed → same result
auto rng = spanny::stochastic::random_context_t{42};
auto planner = spanny::stochastic::rrt_t{rng};

auto result1 = planner(start, goal, context);
// Reset with same seed
rng = spanny::stochastic::random_context_t{42};
auto result2 = planner(start, goal, context);

// result1 and result2 will be identical
```

### Adaptive Planning

Retry with increased limits if planning fails:

```cpp
auto rng = spanny::stochastic::random_context_t{
    std::random_device{}()
};
auto planner = spanny::stochastic::rrt_t{rng};

std::vector<std::size_t> limits{500, 1000, 2000, 5000};

for (auto limit : limits) {
  context.expansion_limit = limit;
  auto result = planner(start, goal, context);

  if (result.has_value()) {
    std::cout << "Success with limit " << limit << "\n";
    return result.value();
  }

  std::cout << "Failed with limit " << limit << ", retrying...\n";
}

std::cerr << "Goal appears unreachable\n";
```

## Custom Random Generators

### Mock Generator for Testing

```cpp
struct mock_random_t {
  double real_between(double min, double max) {
    return min;  // Always return minimum
  }

  bool yes_maybe(double probability) {
    return false;  // Never true
  }
};

// Use in tests
mock_random_t rng;
auto planner = spanny::stochastic::rrt_t{rng};
auto result = planner(start, goal, context);
EXPECT_FALSE(result.has_value());  // Should fail with bad RNG
```

### Custom Distribution

```cpp
struct gaussian_random_t {
  explicit gaussian_random_t(uint32_t seed)
      : gen_{seed}, normal_{0.0, 1.0} {}

  double real_between(double min, double max) {
    // Use Gaussian instead of uniform
    double sample = normal_(gen_);
    double range = max - min;
    return min + (std::tanh(sample) + 1.0) * 0.5 * range;
  }

  bool yes_maybe(double probability) {
    return std::bernoulli_distribution{probability}(gen_);
  }

 private:
  std::mt19937 gen_;
  std::normal_distribution<> normal_;
};

// Samples cluster around center of bounds
auto rng = gaussian_random_t{42};
auto planner = spanny::stochastic::rrt_t{rng};
```

## Performance Characteristics

### Time Complexity

**Per Iteration**:
- Sample: O(1)
- Find nearest: O(n) where n = tree size
- Collision check: O(m) where m = obstacle count
- Add node: O(1)

**Overall**: O(k × n) where:
- k = expansion limit
- n = average tree size during planning

**Bottleneck**: Nearest neighbor search (O(n) per iteration)

For large trees (> 10,000 nodes), consider spatial data structures (KD-tree).

### Space Complexity

**Tree Storage**: O(n) where n = number of nodes

Each node stores:
- `position_t`: 16 bytes (2 doubles)
- `node_id_t`: 8 bytes (size_t)
- Total: ~24 bytes per node

Each edge stores:
- 2 `node_id_t`: 16 bytes
- 1 `double` (cost): 8 bytes
- Total: ~24 bytes per edge

**Example**: 1000-node tree ≈ 48 KB

## Limitations

### Current Implementation

1. **No Path Extraction**: Returns tree, not explicit path from start to goal
2. **Linear Nearest Neighbor**: O(n) search, slow for large trees
3. **No Path Optimization**: First found path returned (not shortest)
4. **Single Goal**: Cannot handle multiple goal regions
5. **Circular Obstacles Only**: No polygons or other shapes

### Workarounds

**Path Extraction**: Traverse edges backwards from goal

**Large Trees**: Use smaller `sample_distance` to reduce node count

**Path Quality**: Use RRT* variant (not implemented) or post-process with smoothing

## See Also

- [Core Types](core-types.md) - Understanding `tree_t`, `node_t`, and `edge_t`
- [Planning Context](planning-context.md) - Configuring planning parameters
- [Testing Utilities](testing-utilities.md) - Mock generators for testing
- [Basic Planning Example](../examples/basic-planning.md) - Complete usage example
- [Doxygen Reference](/doxygen/) - Full API documentation
