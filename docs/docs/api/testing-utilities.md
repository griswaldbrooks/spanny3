---
sidebar_position: 4
---

# Testing Utilities

Tools and patterns for testing RRT path planning code using GoogleTest and GoogleMock.

## Mock Random Generator

The RRT algorithm uses randomness, making it challenging to test deterministically. The mock random generator enables controlled, reproducible tests.

### Definition

```cpp
struct mock_random_t {
  MOCK_METHOD(double, real_between, (double min, double max));
  MOCK_METHOD(bool, yes_maybe, (double probability));
};
```

Satisfies the `some_random_generator` concept required by `rrt_t`.

### Basic Usage

```cpp
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "spanny/rrt.hpp"

TEST(Planning, DeterministicTest) {
  // Create mock
  mock_random_t rng;

  // Set return values
  ON_CALL(rng, real_between).WillByDefault([](auto, auto) {
    return 0.;  // Always return 0
  });

  ON_CALL(rng, yes_maybe).WillByDefault([](auto) {
    return false;  // Always return false
  });

  // Use with planner
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // Verify behavior
  EXPECT_FALSE(result.has_value());
}
```

### Setting Expectations

Control mock behavior with GoogleMock:

```cpp
TEST(Sampling, AlwaysSampleGoal) {
  mock_random_t rng;

  // Always return true (always sample goal)
  ON_CALL(rng, yes_maybe).WillByDefault(testing::Return(true));

  // Set real_between to return midpoint
  ON_CALL(rng, real_between).WillByDefault(
      [](double min, double max) {
        return (min + max) / 2.0;
      });

  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // With goal always sampled, should converge quickly
  ASSERT_TRUE(result.has_value());
}
```

## Test Fixtures

### Planning Test Fixture

Share setup across multiple tests:

```cpp
class PlanningTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Common setup
    start_ = spanny::position_t{0.0, 0.0};
    goal_ = spanny::position_t{5.0, 5.0};

    context_ = spanny::planning_context_t{
        .x_limits = {-10., 10.},
        .y_limits = {-10., 10.},
        .expansion_limit = 1000,
        .sample_distance = 1.,
        .goal_probability = 0.1,
        .obstacles = {}
    };
  }

  spanny::position_t start_;
  spanny::position_t goal_;
  spanny::planning_context_t context_;
};

TEST_F(PlanningTest, BasicPlanning) {
  // Use fixture members
  auto distance = spanny::distance_between(start_, goal_);
  EXPECT_NEAR(distance, 7.07, 0.01);
}

TEST_F(PlanningTest, WithObstacles) {
  // Modify fixture for this test
  context_.obstacles.push_back({2.5, 2.5, 1.0});

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  // ...
}
```

## Custom Matchers

### Tree Comparison Matcher

Compare trees with tolerance for floating-point differences:

```cpp
MATCHER_P2(IsSameTree, expected_tree, tolerance, "") {
  auto const& given_tree = arg;

  // Check node count
  if (expected_tree.nodes.size() != given_tree.nodes.size()) {
    *result_listener << "different node count: "
                     << expected_tree.nodes.size() << " vs "
                     << given_tree.nodes.size();
    return false;
  }

  // Compare nodes
  for (auto const& [expected_node, given_node] :
       std::views::zip(expected_tree.nodes, given_tree.nodes)) {

    if (expected_node.id != given_node.id) {
      *result_listener << "different node ID";
      return false;
    }

    auto dist = spanny::distance_between(expected_node.position,
                                         given_node.position);
    if (dist > tolerance) {
      *result_listener << "node position differs by " << dist;
      return false;
    }
  }

  // Compare edges
  for (auto const& [expected_edge, given_edge] :
       std::views::zip(expected_tree.edges, given_tree.edges)) {

    if (expected_edge.parent != given_edge.parent ||
        expected_edge.child != given_edge.child ||
        std::abs(expected_edge.cost - given_edge.cost) > tolerance) {
      *result_listener << "different edge";
      return false;
    }
  }

  return true;
}
```

**Usage**:

```cpp
TEST(Planning, SameStartEnd) {
  auto point = spanny::position_t{0., 0.};
  auto expected = make_same_start_goal_tree(point);

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(point, point, context);

  ASSERT_TRUE(result.has_value());
  constexpr auto tolerance = 1e-5;
  EXPECT_THAT(result.value(), IsSameTree(expected, tolerance));
}
```

## Test Helpers

### Creating Test Trees

Helper function for edge cases:

```cpp
spanny::tree_t make_same_start_goal_tree(
    spanny::position_t const& point) {

  std::vector<spanny::node_t> nodes{
      spanny::node_t{point},  // Start
      spanny::node_t{point}   // Goal (same position)
  };

  std::vector<spanny::edge_t> edges{
      spanny::edge_t{
          .parent = nodes[0].id,
          .child = nodes[1].id,
          .cost = 0.0  // Zero cost (same position)
      }
  };

  return spanny::tree_t{.nodes = nodes, .edges = edges};
}
```

**Usage**:

```cpp
TEST(Planning, SameStartEnd) {
  auto point = spanny::position_t{5., 5.};

  // Expected tree
  auto expected = make_same_start_goal_tree(point);

  // Plan with same start and goal
  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(point, point, context);

  // Should match expected tree
  ASSERT_TRUE(result.has_value());
  EXPECT_THAT(result.value(), IsSameTree(expected, 1e-5));
}
```

### Obstacle Creation Helpers

Create test obstacles programmatically:

```cpp
std::vector<spanny::circle_t> make_grid_obstacles(
    int rows, int cols, double spacing, double radius) {

  std::vector<spanny::circle_t> obstacles;
  obstacles.reserve(rows * cols);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      obstacles.push_back({
          .x = i * spacing,
          .y = j * spacing,
          .radius = radius
      });
    }
  }

  return obstacles;
}

std::vector<spanny::circle_t> make_corridor_obstacles(
    double width, double gap_size) {

  return {
      {0., width / 2 + gap_size / 2, width / 2},   // Top wall
      {0., -(width / 2 + gap_size / 2), width / 2} // Bottom wall
  };
}
```

**Usage**:

```cpp
TEST(Planning, GridObstacles) {
  auto obstacles = make_grid_obstacles(5, 5, 2.0, 0.5);
  context_.obstacles = obstacles;

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  ASSERT_TRUE(result.has_value());
}

TEST(Planning, NarrowCorridor) {
  auto obstacles = make_corridor_obstacles(2.0, 0.5);
  context_.obstacles = obstacles;

  // Requires small sample distance
  context_.sample_distance = 0.1;

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  ASSERT_TRUE(result.has_value());
}
```

## Testing Patterns

### Test Failure Cases

```cpp
TEST(Planning, UnreachableGoal) {
  // Surround goal with obstacles
  auto goal = spanny::position_t{5., 5.};
  std::vector<spanny::circle_t> obstacles{
      {5., 5., 0.5}  // Obstacle at goal
  };

  context_.obstacles = obstacles;

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // Should fail
  EXPECT_FALSE(result.has_value());
  EXPECT_THAT(result.error(),
              testing::HasSubstr("failed to reach goal"));
}
```

### Test Edge Cases

```cpp
TEST(Planning, ZeroExpansionLimit) {
  context_.expansion_limit = 0;

  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // Should fail immediately
  EXPECT_FALSE(result.has_value());
}

TEST(Planning, VerySmallSampleDistance) {
  context_.sample_distance = 0.001;  // Very small
  context_.expansion_limit = 100000;  // Compensate

  mock_random_t rng;
  // Set up mock to sample reasonably
  ON_CALL(rng, real_between).WillByDefault(
      [](double min, double max) {
        return (min + max) / 2.0;
      });

  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // May succeed but needs many nodes
  if (result.has_value()) {
    EXPECT_GT(result.value().nodes.size(), 1000);
  }
}
```

### Test with Real Random Generator

Test end-to-end with actual randomness:

```cpp
TEST(Planning, RealRandomGenerator) {
  // Use real random generator with fixed seed
  auto rng = spanny::stochastic::random_context_t{42};
  auto planner = spanny::stochastic::rrt_t{rng};

  auto result = planner(start, goal, context);

  // With open space and reasonable limits, should succeed
  ASSERT_TRUE(result.has_value());

  auto const& tree = result.value();
  EXPECT_GT(tree.nodes.size(), 2);  // At least start and goal
  EXPECT_EQ(tree.edges.size(), tree.nodes.size() - 1);  // Tree property
}
```

## Debugging Test Failures

### Print Tree Structure

```cpp
void print_tree(spanny::tree_t const& tree) {
  std::cout << "Nodes (" << tree.nodes.size() << "):\n";
  for (auto const& node : tree.nodes) {
    std::cout << "  ID " << node.id << ": ("
              << node.position.x << ", " << node.position.y << ")\n";
  }

  std::cout << "Edges (" << tree.edges.size() << "):\n";
  for (auto const& edge : tree.edges) {
    std::cout << "  " << edge.parent << " -> " << edge.child
              << " (cost: " << edge.cost << ")\n";
  }
}

TEST(Planning, DebugTree) {
  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  if (result.has_value()) {
    print_tree(result.value());
  } else {
    std::cout << "Error: " << result.error() << "\n";
  }

  ASSERT_TRUE(result.has_value());
}
```

### Verify Mock Calls

Check that mock is called as expected:

```cpp
TEST(Planning, MockCallVerification) {
  mock_random_t rng;

  // Expect real_between called multiple times
  EXPECT_CALL(rng, real_between)
      .Times(testing::at least(10));

  // Expect yes_maybe called for each sample
  EXPECT_CALL(rng, yes_maybe)
      .Times(testing::at least(1));

  auto planner = spanny::stochastic::rrt_t{rng};
  planner(start, goal, context);

  // Expectations verified at test end
}
```

## Best Practices

1. **Use mocks for unit tests**: Deterministic behavior
2. **Use real RNG for integration tests**: End-to-end validation
3. **Fixed seeds for reproducibility**: Same input → same output
4. **Test both success and failure**: Verify error handling
5. **Test edge cases**: Boundaries, limits, special values
6. **Use custom matchers**: Readable assertions
7. **Create test helpers**: Reduce duplication
8. **Print debug info**: Aid troubleshooting
9. **Verify mock calls**: Ensure algorithm behaves correctly
10. **Measure coverage**: Aim for high code coverage

## Example Test Suite

```cpp
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "spanny/rrt.hpp"

namespace spanny {

// Test fixture
class RRTTest : public ::testing::Test {
 protected:
  void SetUp() override {
    start_ = position_t{0., 0.};
    goal_ = position_t{5., 5.};
    context_ = planning_context_t{
        .x_limits = {-10., 10.},
        .y_limits = {-10., 10.},
        .expansion_limit = 1000,
        .sample_distance = 1.,
        .goal_probability = 0.1,
        .obstacles = {}
    };
  }

  position_t start_;
  position_t goal_;
  planning_context_t context_;
};

// Test same start and goal
TEST_F(RRTTest, SameStartGoal) {
  mock_random_t rng;
  auto planner = stochastic::rrt_t{rng};

  auto result = planner(start_, start_, context_);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value().nodes.size(), 2);
}

// Test with obstacles
TEST_F(RRTTest, WithObstacles) {
  context_.obstacles.push_back({2.5, 2.5, 1.0});

  auto rng = stochastic::random_context_t{42};
  auto planner = stochastic::rrt_t{rng};

  auto result = planner(start_, goal_, context_);

  ASSERT_TRUE(result.has_value());
}

// Test failure
TEST_F(RRTTest, UnreachableGoal) {
  // Block goal
  context_.obstacles.push_back({5., 5., 0.5});

  mock_random_t rng;
  auto planner = stochastic::rrt_t{rng};

  auto result = planner(start_, goal_, context_);

  EXPECT_FALSE(result.has_value());
}

}  // namespace spanny
```

## See Also

- [Testing Guide](../development/testing-guide.md) - Complete testing workflow
- [RRT Planner](rrt-planner.md) - Understanding the planner
- [GoogleTest Primer](https://google.github.io/googletest/primer.html) - GoogleTest documentation
- [GoogleMock Cheat Sheet](https://google.github.io/googletest/gmock_cheat_sheet.html) - Mock syntax reference
