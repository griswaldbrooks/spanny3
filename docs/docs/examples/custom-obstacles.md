---
sidebar_position: 2
---

# Custom Obstacles

Learn how to create and configure obstacle scenarios for path planning.

## JSON Scenario Format

Scenarios are defined in JSON files with this structure:

```json
{
  "expansion_limit": 1000,
  "sample_distance": 0.1,
  "goal_probability": 0.1,
  "x_limits": [-0.5, 0.5],
  "y_limits": [-0.5, 0.5],
  "start": [-0.4, -0.4],
  "goal": [0.4, 0.4],
  "obstacles": [
    [0, 0, 0.1],
    [0.3, 0.2, 0.1]
  ]
}
```

### Field Descriptions

| Field | Type | Description |
|-------|------|-------------|
| `expansion_limit` | integer | Maximum nodes before giving up |
| `sample_distance` | number | Step size for tree expansion |
| `goal_probability` | number | Probability (0-1) of sampling goal |
| `x_limits` | [min, max] | X-axis boundaries |
| `y_limits` | [min, max] | Y-axis boundaries |
| `start` | [x, y] | Starting position |
| `goal` | [x, y] | Goal position |
| `obstacles` | array | List of circular obstacles |

### Obstacle Format

Each obstacle is a 3-element array: `[x, y, radius]`

```json
"obstacles": [
  [0, 0, 0.1],      // Circle at origin, radius 0.1
  [0.3, 0.2, 0.15], // Circle at (0.3, 0.2), radius 0.15
  [-0.2, 0.3, 0.2]  // Circle at (-0.2, 0.3), radius 0.2
]
```

## Running with Custom Scenarios

### Using rrt_cli

Run the CLI tool with a scenario file:

```bash
pixi run build
./build/rrt_cli --scenario config/my_scenario.json
```

Or use the configured task:

```bash
pixi run run-scenario  # Uses config/scenario.json
```

### CLI Output

The tool outputs node positions:

```
-0.400, -0.400
-0.365, -0.365
-0.330, -0.330
...
0.365, 0.365
0.400, 0.400
```

Redirect to a file for visualization:

```bash
./build/rrt_cli --scenario config/my_scenario.json > path.csv
```

## Example Scenarios

### Simple Corridor

Navigate through a narrow passage:

```json
{
  "expansion_limit": 2000,
  "sample_distance": 0.05,
  "goal_probability": 0.1,
  "x_limits": [-1.0, 1.0],
  "y_limits": [-1.0, 1.0],
  "start": [-0.8, 0.0],
  "goal": [0.8, 0.0],
  "obstacles": [
    [-0.3, 0.5, 0.3],
    [-0.3, -0.5, 0.3],
    [0.3, 0.5, 0.3],
    [0.3, -0.5, 0.3]
  ]
}
```

Creates a corridor between obstacles. Small `sample_distance` helps navigate tight spaces.

### Obstacle Field

Navigate through scattered obstacles:

```json
{
  "expansion_limit": 5000,
  "sample_distance": 0.1,
  "goal_probability": 0.05,
  "x_limits": [-5.0, 5.0],
  "y_limits": [-5.0, 5.0],
  "start": [-4.0, -4.0],
  "goal": [4.0, 4.0],
  "obstacles": [
    [0, 0, 0.8],
    [2, 1, 0.6],
    [-2, 1, 0.6],
    [1, -2, 0.7],
    [-1, -2, 0.7],
    [3, 3, 0.5],
    [-3, 3, 0.5],
    [3, -3, 0.5],
    [-3, -3, 0.5]
  ]
}
```

Larger search space with multiple obstacles. Higher `expansion_limit` handles complexity.

### Maze-Like Environment

Dense obstacle placement:

```json
{
  "expansion_limit": 10000,
  "sample_distance": 0.05,
  "goal_probability": 0.15,
  "x_limits": [-2.0, 2.0],
  "y_limits": [-2.0, 2.0],
  "start": [-1.8, -1.8],
  "goal": [1.8, 1.8],
  "obstacles": [
    [-1.0, 0, 0.3],
    [0, 1.0, 0.3],
    [1.0, 0, 0.3],
    [0, -1.0, 0.3],
    [-0.5, 0.5, 0.2],
    [0.5, 0.5, 0.2],
    [0.5, -0.5, 0.2],
    [-0.5, -0.5, 0.2]
  ]
}
```

Small steps and high goal probability help in complex environments.

### Open Space (Baseline)

No obstacles for performance comparison:

```json
{
  "expansion_limit": 1000,
  "sample_distance": 0.5,
  "goal_probability": 0.05,
  "x_limits": [-10.0, 10.0],
  "y_limits": [-10.0, 10.0],
  "start": [0.0, 0.0],
  "goal": [5.0, 5.0],
  "obstacles": []
}
```

## Creating Obstacles Programmatically

### In C++

Create obstacles in code:

```cpp
#include "spanny/rrt.hpp"
#include <vector>

std::vector<spanny::circle_t> create_grid_obstacles(
    int rows, int cols, double spacing, double radius) {

  std::vector<spanny::circle_t> obstacles;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      double x = i * spacing;
      double y = j * spacing;
      obstacles.push_back({x, y, radius});
    }
  }

  return obstacles;
}

int main() {
  // Create 5x5 grid of obstacles
  auto obstacles = create_grid_obstacles(5, 5, 2.0, 0.3);

  spanny::planning_context_t context{
      .x_limits = {-1.0, 11.0},
      .y_limits = {-1.0, 11.0},
      .expansion_limit = 5000,
      .sample_distance = 0.2,
      .goal_probability = 0.1,
      .obstacles = obstacles
  };

  // Plan path
  auto start = spanny::position_t{0.0, 0.0};
  auto goal = spanny::position_t{10.0, 10.0};

  auto random_generator = spanny::stochastic::random_context_t{42};
  auto planner = spanny::stochastic::rrt_t{random_generator};

  auto result = planner(start, goal, context);
  // ...
}
```

### Random Obstacles

Generate random obstacle fields:

```cpp
#include <random>

std::vector<spanny::circle_t> create_random_obstacles(
    int count,
    spanny::bounds_t x_bounds,
    spanny::bounds_t y_bounds,
    double min_radius,
    double max_radius,
    unsigned seed = std::random_device{}()) {

  std::mt19937 gen{seed};
  std::uniform_real_distribution<> x_dist{x_bounds.min, x_bounds.max};
  std::uniform_real_distribution<> y_dist{y_bounds.min, y_bounds.max};
  std::uniform_real_distribution<> r_dist{min_radius, max_radius};

  std::vector<spanny::circle_t> obstacles;
  obstacles.reserve(count);

  for (int i = 0; i < count; ++i) {
    obstacles.push_back({
        .x = x_dist(gen),
        .y = y_dist(gen),
        .radius = r_dist(gen)
    });
  }

  return obstacles;
}

// Usage
auto obstacles = create_random_obstacles(
    50,                      // 50 obstacles
    {-10.0, 10.0},          // X range
    {-10.0, 10.0},          // Y range
    0.3,                    // Min radius
    0.8,                    // Max radius
    42                      // Seed for reproducibility
);
```

## Collision Detection

Understanding how obstacles affect planning:

### Line-Circle Intersection

The planner checks if edges intersect obstacles using quadratic formula:

```cpp
bool in_collision(position_t const& p1,
                  position_t const& p2,
                  std::span<circle_t const> obstacles);
```

Returns `true` if line segment from `p1` to `p2` intersects any obstacle.

### Point-Circle Collision

Check if a position is inside an obstacle:

```cpp
bool in_collision(position_t const& position,
                  std::span<circle_t const> obstacles);
```

### Performance Implications

Collision checking scales linearly with obstacle count:
- 10 obstacles: ~1 microsecond per check
- 100 obstacles: ~10 microseconds per check

For large obstacle counts (1000+), consider spatial data structures.

## Testing Configurations

### Verify Scenario Validity

Check if start/goal are collision-free:

```cpp
#include "spanny/rrt.hpp"
#include <iostream>

bool is_valid_scenario(
    spanny::position_t const& start,
    spanny::position_t const& goal,
    std::vector<spanny::circle_t> const& obstacles) {

  if (spanny::in_collision(start, obstacles)) {
    std::cerr << "Start position is in collision\n";
    return false;
  }

  if (spanny::in_collision(goal, obstacles)) {
    std::cerr << "Goal position is in collision\n";
    return false;
  }

  return true;
}
```

### Reachability Testing

Not all scenarios are solvable. Test with increasing limits:

```cpp
void test_reachability(
    spanny::position_t const& start,
    spanny::position_t const& goal,
    spanny::planning_context_t context) {

  std::vector<std::size_t> limits{100, 500, 1000, 5000, 10000};

  for (auto limit : limits) {
    context.expansion_limit = limit;

    auto random_generator = spanny::stochastic::random_context_t{42};
    auto planner = spanny::stochastic::rrt_t{random_generator};

    auto result = planner(start, goal, context);

    if (result.has_value()) {
      std::cout << "Success with limit " << limit
                << " (" << result.value().nodes.size() << " nodes)\n";
      return;
    }
  }

  std::cout << "Goal unreachable with limits up to "
            << limits.back() << "\n";
}
```

## Debugging Scenarios

### Visualize Obstacles

Print obstacle positions for plotting:

```cpp
void print_obstacles(std::vector<spanny::circle_t> const& obstacles) {
  std::cout << "Obstacles:\n";
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    auto const& obs = obstacles[i];
    std::cout << "  " << i << ": center=(" << obs.x << ", " << obs.y
              << "), radius=" << obs.radius << "\n";
  }
}
```

### Check Obstacle Coverage

Calculate how much of the space is blocked:

```cpp
double calculate_obstacle_coverage(
    std::vector<spanny::circle_t> const& obstacles,
    spanny::bounds_t x_bounds,
    spanny::bounds_t y_bounds) {

  double total_area = (x_bounds.max - x_bounds.min) *
                     (y_bounds.max - y_bounds.min);

  double obstacle_area = 0.0;
  for (auto const& obs : obstacles) {
    obstacle_area += M_PI * obs.radius * obs.radius;
  }

  return obstacle_area / total_area;
}

// If coverage > 0.5, planning may be very difficult
```

### Estimate Planning Difficulty

Simple heuristic for scenario complexity:

```cpp
enum class Difficulty { EASY, MEDIUM, HARD, VERY_HARD };

Difficulty estimate_difficulty(
    spanny::position_t const& start,
    spanny::position_t const& goal,
    std::vector<spanny::circle_t> const& obstacles,
    spanny::bounds_t x_bounds,
    spanny::bounds_t y_bounds) {

  double distance = spanny::distance_between(start, goal);
  double coverage = calculate_obstacle_coverage(obstacles, x_bounds, y_bounds);
  double density = obstacles.size() /
                  ((x_bounds.max - x_bounds.min) *
                   (y_bounds.max - y_bounds.min));

  if (coverage < 0.1 && density < 1.0) {
    return Difficulty::EASY;
  } else if (coverage < 0.3 && density < 3.0) {
    return Difficulty::MEDIUM;
  } else if (coverage < 0.5 && density < 5.0) {
    return Difficulty::HARD;
  } else {
    return Difficulty::VERY_HARD;
  }
}
```

## Best Practices

1. **Start simple**: Test with no obstacles first
2. **Incremental complexity**: Add obstacles gradually
3. **Validate positions**: Ensure start/goal aren't in collision
4. **Tune parameters**: Adjust `sample_distance` based on obstacle density
5. **Check reachability**: Some scenarios may be unsolvable
6. **Use fixed seeds**: Reproducible results aid debugging
7. **Visualize**: Plot obstacles and resulting paths
8. **Measure coverage**: High obstacle coverage requires more exploration

## Next Steps

- [Basic Planning](basic-planning.md) - Introduction to RRT planning
- [Benchmarking](benchmarking.md) - Measure scenario performance
- [API: Planning Context](../api/planning-context.md) - Parameter reference
- [API: Core Types](../api/core-types.md) - Understanding obstacles and positions
