---
sidebar_position: 2
---

# Planning Context

Configuration parameters for RRT path planning algorithm.

## Overview

The `planning_context_t` structure defines all parameters needed for RRT planning:

```cpp
struct planning_context_t {
  bounds_t x_limits;               // Minimum and maximum x-values
  bounds_t y_limits;               // Minimum and maximum y-values
  std::size_t expansion_limit;     // Maximum number of nodes to expand
  double sample_distance;          // Fixed distance to project when extending
  double goal_probability;         // Probability of sampling the goal
  std::vector<circle_t> obstacles; // Obstacles to avoid
};
```

## Parameters

### x_limits and y_limits

Define the rectangular boundaries of the planning space.

```cpp
bounds_t x_limits;  // {min, max} for x-axis
bounds_t y_limits;  // {min, max} for y-axis
```

**Type**: `bounds_t` (struct with `min` and `max` fields)

**Purpose**: Constrain random sampling to valid region

**Example**:
```cpp
auto context = spanny::planning_context_t{
    .x_limits = {-10.0, 10.0},  // X from -10 to +10
    .y_limits = {-10.0, 10.0},  // Y from -10 to +10
    // ...
};
```

**Guidelines**:
- Tight bounds speed up planning by focusing search
- Bounds should fully contain start, goal, and valid paths
- Avoid unnecessarily large bounds (wastes sampling effort)

**Common Patterns**:

Centered around origin:
```cpp
.x_limits = {-10.0, 10.0},
.y_limits = {-10.0, 10.0},
```

Asymmetric workspace:
```cpp
.x_limits = {0.0, 20.0},   // Only positive X
.y_limits = {-5.0, 5.0},   // Narrow Y range
```

Minimal bounding box:
```cpp
// For start (1, 2) and goal (8, 9)
.x_limits = {0.0, 10.0},   // Slight margin
.y_limits = {1.0, 10.0},
```

---

### expansion_limit

Maximum number of nodes to add to the tree before giving up.

```cpp
std::size_t expansion_limit;
```

**Type**: `std::size_t` (unsigned integer)

**Purpose**: Prevent infinite loops when goal is unreachable

**Default Recommendation**: 1000 for simple scenarios

**Example**:
```cpp
.expansion_limit = 1000,  // Try up to 1000 nodes
```

**Tuning Guidelines**:

| Scenario Complexity | Suggested Limit |
|---------------------|-----------------|
| Open space | 100 - 500 |
| Few obstacles (< 5) | 500 - 2000 |
| Many obstacles (5-20) | 2000 - 5000 |
| Dense obstacles (> 20) | 5000 - 10000 |
| Maze-like | 10000+ |

**Trade-offs**:
- **Too low**: May fail to find valid paths
- **Too high**: Wastes time on unsolvable problems
- **Just right**: Balances success rate vs. runtime

**Adaptive Strategy**:
```cpp
// Start low, increase if needed
for (auto limit : {500, 2000, 5000}) {
  context.expansion_limit = limit;
  auto result = planner(start, goal, context);
  if (result.has_value()) {
    return result;
  }
}
```

---

### sample_distance

Fixed step size when extending the tree toward a sample.

```cpp
double sample_distance;
```

**Type**: `double` (positive value)

**Purpose**: Controls tree granularity and growth rate

**Default Recommendation**: 0.5 for workspace size ~20 units

**Example**:
```cpp
.sample_distance = 0.5,  // Extend by 0.5 units per step
```

**Tuning Guidelines**:

Scale with workspace size:
```cpp
// Rule of thumb: ~1-5% of workspace dimension
double workspace_size = x_limits.max - x_limits.min;
double sample_distance = workspace_size * 0.02;  // 2%
```

| Workspace Size | Suggested Distance |
|----------------|-------------------|
| 1 x 1 | 0.01 - 0.05 |
| 10 x 10 | 0.1 - 0.5 |
| 100 x 100 | 1.0 - 5.0 |

**Effect on Planning**:

**Large distance** (e.g., 2.0):
- Pros: Fewer nodes needed, faster planning
- Cons: Coarse paths, may miss narrow passages

**Small distance** (e.g., 0.1):
- Pros: Smooth paths, handles tight spaces
- Cons: More nodes needed, slower planning

**Obstacle Density**:
```cpp
// Dense obstacles → small steps
.sample_distance = 0.1,

// Open space → large steps
.sample_distance = 1.0,
```

**Narrow Passages**:

For corridors width `w`, use:
```cpp
.sample_distance = w / 4,  // Quarter of passage width
```

Example for 0.5 unit corridor:
```cpp
.sample_distance = 0.125,
```

---

### goal_probability

Probability of sampling the goal instead of a random position.

```cpp
double goal_probability;
```

**Type**: `double` (range: 0.0 to 1.0)

**Purpose**: Balance exploration vs. exploitation

**Default Recommendation**: 0.05 (5%)

**Example**:
```cpp
.goal_probability = 0.05,  // 5% chance per sample
```

**Tuning Guidelines**:

| Scenario | Suggested Probability |
|----------|----------------------|
| Open space | 0.01 - 0.05 |
| Few obstacles | 0.05 - 0.10 |
| Dense obstacles | 0.10 - 0.20 |
| Very difficult | 0.20 - 0.30 |

**Behavior**:

**Low probability** (e.g., 0.01):
- More exploration
- Better coverage
- May find better paths
- Slower convergence

**High probability** (e.g., 0.5):
- Direct goal-seeking
- Faster convergence
- Less exploration
- May miss good paths

**Extreme values**:

Never sample goal:
```cpp
.goal_probability = 0.0,  // Pure exploration (may not converge)
```

Always sample goal:
```cpp
.goal_probability = 1.0,  // Pure exploitation (limited exploration)
```

**Adaptive Probability**:

Increase probability over time:
```cpp
// Start: low (explore)
// Later: high (exploit)
double progress = tree.nodes.size() / expansion_limit;
double adaptive_prob = 0.05 + 0.15 * progress;  // 5% → 20%
```

---

### obstacles

List of circular obstacles to avoid during planning.

```cpp
std::vector<circle_t> obstacles;
```

**Type**: `std::vector<circle_t>` (can be empty)

**Purpose**: Define collision geometry

**Example**:
```cpp
.obstacles = {
    {2.0, 2.0, 1.0},    // Center (2, 2), radius 1.0
    {5.0, 3.0, 1.5},    // Center (5, 3), radius 1.5
    {3.0, 6.0, 1.0},    // Center (3, 6), radius 1.0
}
```

**Obstacle Format**:

Each obstacle is a `circle_t`:
```cpp
struct circle_t {
  double x;       // X coordinate of center
  double y;       // Y coordinate of center
  double radius;  // Radius of circle
};
```

**No Obstacles**:
```cpp
.obstacles = {}  // Empty vector
```

**Creating Obstacles**:

Single obstacle:
```cpp
auto obstacle = spanny::circle_t{
    .x = 5.0,
    .y = 5.0,
    .radius = 2.0
};

std::vector<spanny::circle_t> obstacles{obstacle};
```

Multiple obstacles:
```cpp
std::vector<spanny::circle_t> obstacles{
    {0.0, 0.0, 1.0},
    {3.0, 3.0, 0.5},
    {-2.0, 4.0, 1.5}
};
```

**Performance Considerations**:

Collision checking scales O(n) with obstacle count:
- 10 obstacles: ~1 μs per check
- 100 obstacles: ~10 μs per check
- 1000 obstacles: ~100 μs per check

For 1000+ obstacles, consider spatial indexing (not currently implemented).

**See Also**: [Custom Obstacles](../examples/custom-obstacles.md)

## Complete Example

```cpp
#include "spanny/rrt.hpp"
#include <random>

int main() {
  // Define scenario
  auto start = spanny::position_t{-8.0, -8.0};
  auto goal = spanny::position_t{8.0, 8.0};

  // Configure planning context
  spanny::planning_context_t context{
      // Workspace boundaries
      .x_limits = {-10.0, 10.0},
      .y_limits = {-10.0, 10.0},

      // Algorithm parameters
      .expansion_limit = 2000,        // Up to 2000 nodes
      .sample_distance = 0.3,         // Small steps for obstacles
      .goal_probability = 0.08,       // 8% goal bias

      // Collision geometry
      .obstacles = {
          {0.0, 0.0, 2.0},           // Large central obstacle
          {-4.0, 4.0, 1.0},          // Upper left
          {4.0, -4.0, 1.0},          // Lower right
          {-4.0, -4.0, 0.8},         // Lower left
          {4.0, 4.0, 0.8}            // Upper right
      }
  };

  // Create planner
  auto random_generator = spanny::stochastic::random_context_t{42};
  auto planner = spanny::stochastic::rrt_t{random_generator};

  // Plan path
  auto result = planner(start, goal, context);

  if (result.has_value()) {
    std::cout << "Success! Tree size: "
              << result.value().nodes.size() << "\n";
  } else {
    std::cerr << "Failed: " << result.error() << "\n";
  }

  return 0;
}
```

## Parameter Tuning Workflow

### 1. Start with Defaults

```cpp
spanny::planning_context_t context{
    .x_limits = {-10.0, 10.0},
    .y_limits = {-10.0, 10.0},
    .expansion_limit = 1000,
    .sample_distance = 0.5,
    .goal_probability = 0.05,
    .obstacles = /* your obstacles */
};
```

### 2. Adjust for Scenario

**If planning fails**:
- Increase `expansion_limit` (2x)
- Increase `goal_probability` (try 0.10)
- Decrease `sample_distance` (if tight spaces)

**If planning is slow**:
- Decrease `expansion_limit` (if successful)
- Increase `sample_distance` (if no narrow passages)
- Tighten `x_limits` and `y_limits`

**If path is too coarse**:
- Decrease `sample_distance`
- May increase planning time proportionally

### 3. Validate Configuration

```cpp
// Check start/goal validity
if (spanny::in_collision(start, context.obstacles)) {
  std::cerr << "Start is in collision!\n";
}

// Check bounds
if (start.x < context.x_limits.min ||
    start.x > context.x_limits.max) {
  std::cerr << "Start outside bounds!\n";
}

// Estimate difficulty
double workspace_diagonal = std::hypot(
    context.x_limits.max - context.x_limits.min,
    context.y_limits.max - context.y_limits.min
);
double path_distance = spanny::distance_between(start, goal);
double relative_distance = path_distance / workspace_diagonal;

if (relative_distance > 0.5) {
  std::cout << "Long path - consider increasing expansion_limit\n";
}
```

## Common Configurations

### Indoor Navigation

Small, cluttered space with many obstacles:

```cpp
spanny::planning_context_t indoor_context{
    .x_limits = {0.0, 10.0},        // 10m x 10m room
    .y_limits = {0.0, 10.0},
    .expansion_limit = 5000,         // Many obstacles need exploration
    .sample_distance = 0.2,          // Small steps for furniture
    .goal_probability = 0.15,        // Higher bias in cluttered space
    .obstacles = /* many small obstacles */
};
```

### Outdoor Open Space

Large, sparse environment:

```cpp
spanny::planning_context_t outdoor_context{
    .x_limits = {-100.0, 100.0},    // 200m x 200m area
    .y_limits = {-100.0, 100.0},
    .expansion_limit = 1000,         // Few obstacles need less nodes
    .sample_distance = 5.0,          // Large steps in open space
    .goal_probability = 0.02,        // Low bias for exploration
    .obstacles = /* few large obstacles */
};
```

### Narrow Passage

Requires precise navigation:

```cpp
spanny::planning_context_t passage_context{
    .x_limits = {-5.0, 5.0},
    .y_limits = {-5.0, 5.0},
    .expansion_limit = 3000,         // Difficult scenario
    .sample_distance = 0.1,          // Very small steps
    .goal_probability = 0.20,        // High bias to push through
    .obstacles = /* creates narrow corridor */
};
```

## See Also

- [Core Types](core-types.md) - Understanding `bounds_t` and `circle_t`
- [RRT Planner](rrt-planner.md) - Using the planner with context
- [Basic Planning](../examples/basic-planning.md) - Complete examples
- [Custom Obstacles](../examples/custom-obstacles.md) - Configuring obstacles
