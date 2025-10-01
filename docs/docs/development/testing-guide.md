---
sidebar_position: 2
---

# Testing Guide

Learn how to run, write, and debug tests for Spanny3 using GoogleTest and Pixi.

## Running Tests

### Quick Test

Run all tests with a single command:

```bash
pixi run test
```

This automatically builds the project if needed and runs the test suite.

### Verbose Output

See detailed test output:

```bash
pixi run test-verbose
```

### Running Specific Tests

Use GoogleTest filters:

```bash
# Run tests matching a pattern
./build/test_rrt --gtest_filter=TreeGeneration.*

# Run single test
./build/test_rrt --gtest_filter=Planning.SameStartEnd

# Exclude tests
./build/test_rrt --gtest_filter=-*BadRandom*
```

### Test Coverage

Generate coverage reports:

```bash
pixi run coverage
```

View results:
- **HTML Report**: `build/coverage/reports/html/index.html`
- **Text Summary**: `build/coverage/reports/coverage.txt`

## Test Structure

### Test Organization

Tests are located in `test/test_rrt.cpp` and organized by component:

```cpp
// Test namespace for fixtures and utilities
namespace spanny {

TEST(TreeGeneration, BadRandom) {
  // Test RRT behavior with faulty random generator
}

TEST(Planning, SameStartEnd) {
  // Test edge case where start equals goal
}

}  // namespace spanny
```

### Test Naming Convention

Follow the pattern: `TEST(TestSuite, TestCase)`

- **TestSuite**: Component or feature being tested (e.g., `TreeGeneration`, `Planning`, `CollisionDetection`)
- **TestCase**: Specific scenario or behavior (e.g., `BadRandom`, `SameStartEnd`, `LineIntersectsCircle`)

## Writing Tests

### Basic Test Structure

```cpp
#include <gtest/gtest.h>
#include "spanny/rrt.hpp"

TEST(MyTestSuite, MyTestCase) {
  // GIVEN - Setup test conditions
  auto start = spanny::position_t{0.0, 0.0};
  auto goal = spanny::position_t{5.0, 5.0};

  // WHEN - Perform action being tested
  auto distance = spanny::distance_between(start, goal);

  // THEN - Verify expected behavior
  EXPECT_NEAR(distance, 7.07, 0.01);
}
```

### Assertions

GoogleTest provides two types of assertions:

**EXPECT**: Test continues after failure
```cpp
EXPECT_EQ(actual, expected);      // Equality
EXPECT_NE(actual, expected);      // Inequality
EXPECT_LT(actual, expected);      // Less than
EXPECT_LE(actual, expected);      // Less than or equal
EXPECT_GT(actual, expected);      // Greater than
EXPECT_GE(actual, expected);      // Greater than or equal
EXPECT_NEAR(actual, expected, tolerance);  // Floating point
EXPECT_TRUE(condition);
EXPECT_FALSE(condition);
```

**ASSERT**: Test stops after failure
```cpp
ASSERT_TRUE(result.has_value());  // Stop test if result is error
auto const& value = result.value();  // Safe to use value
```

### Testing with std::expected

Spanny3 uses `std::expected<T, std::string>` for error handling:

```cpp
TEST(Planning, ValidScenario) {
  // Setup
  mock_random_t rng;
  auto planner = spanny::stochastic::rrt_t{rng};

  // Execute
  auto result = planner(start, goal, context);

  // Verify success
  ASSERT_TRUE(result.has_value()) << "Expected successful planning";
  auto const& tree = result.value();

  // Check tree properties
  EXPECT_GT(tree.nodes.size(), 1);
}

TEST(Planning, InvalidBounds) {
  // Test error case
  auto result = planner(start, goal, bad_context);

  // Verify failure
  EXPECT_FALSE(result.has_value());
  EXPECT_THAT(result.error(), testing::HasSubstr("collision"));
}
```

## Mocking Strategies

### Testing Stochastic Algorithms

RRT is a stochastic algorithm that uses random sampling. Testing requires controlling randomness.

### Mock Random Generator

Use GoogleMock to create deterministic random generators:

```cpp
#include <gmock/gmock.h>

struct mock_random_t {
  MOCK_METHOD(double, real_between, (double min, double max));
  MOCK_METHOD(bool, yes_maybe, (double probability));
};
```

### Setting Mock Behavior

Control what the mock returns:

```cpp
TEST(TreeGeneration, ControlledRandom) {
  // Create mock
  mock_random_t rng;

  // Always return 0 for real_between
  ON_CALL(rng, real_between).WillByDefault([](auto, auto) {
    return 0.;
  });

  // Always return false for yes_maybe
  ON_CALL(rng, yes_maybe).WillByDefault([](auto) {
    return false;
  });

  // Use mock with RRT
  auto planner = spanny::stochastic::rrt_t{rng};
  auto result = planner(start, goal, context);

  // Result is deterministic
  EXPECT_FALSE(result.has_value());
}
```

### Sequence of Return Values

Return different values on successive calls:

```cpp
TEST(Sampling, MultipleAttempts) {
  mock_random_t rng;

  // First call returns 0, second returns 5, then cycle
  ON_CALL(rng, real_between)
      .WillByDefault(testing::Return(0.))
      .WillOnce(testing::Return(5.))
      .WillRepeatedly(testing::Return(0.));

  // Test behavior with varying random values
  auto planner = spanny::stochastic::rrt_t{rng};
  // ...
}
```

### Verifying Mock Calls

Check that functions were called:

```cpp
TEST(Sampling, CallsRandomGenerator) {
  mock_random_t rng;

  // Expect real_between to be called at least once
  EXPECT_CALL(rng, real_between).Times(testing::at least(1));

  auto planner = spanny::stochastic::rrt_t{rng};
  planner(start, goal, context);

  // Mock verification happens automatically at test end
}
```

## Custom Matchers

### Tree Comparison Matcher

The test suite includes a custom matcher for comparing trees:

```cpp
MATCHER_P2(IsSameTree, expected_tree, tolerance, "") {
  auto const& given_tree = arg;

  // Compare nodes
  for (auto const& [expected_node, given_node] :
       std::views::zip(expected_tree.nodes, given_tree.nodes)) {
    if (expected_node.id != given_node.id) {
      return false;
    }
    if (spanny::distance_between(expected_node.position,
                                  given_node.position) > tolerance) {
      return false;
    }
  }

  // Compare edges
  for (auto const& [expected_edge, given_edge] :
       std::views::zip(expected_tree.edges, given_tree.edges)) {
    if (expected_edge.parent != given_edge.parent ||
        expected_edge.child != given_edge.child ||
        expected_edge.cost != given_edge.cost) {
      return false;
    }
  }

  return true;
}
```

Usage:

```cpp
TEST(Planning, SameStartEnd) {
  // Create expected tree
  auto expected = make_same_start_goal_tree(position);

  // Run planner
  auto result = planner(position, position, context);

  // Compare with tolerance
  constexpr auto tolerance = 1e-5;
  ASSERT_TRUE(result.has_value());
  EXPECT_THAT(result.value(), IsSameTree(expected, tolerance));
}
```

### Creating Custom Matchers

Define your own matchers for domain-specific assertions:

```cpp
MATCHER_P(IsValidTree, max_nodes, "") {
  auto const& tree = arg;

  if (tree.nodes.empty()) {
    *result_listener << "tree has no nodes";
    return false;
  }

  if (tree.nodes.size() > max_nodes) {
    *result_listener << "tree has " << tree.nodes.size()
                     << " nodes, expected <= " << max_nodes;
    return false;
  }

  if (tree.edges.size() >= tree.nodes.size()) {
    *result_listener << "tree has more edges than nodes";
    return false;
  }

  return true;
}

// Usage
EXPECT_THAT(tree, IsValidTree(1000));
```

## Test Fixtures

### Basic Fixture

For tests that share setup:

```cpp
class PlanningTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Common setup for all tests
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
  // ...
}
```

### Parameterized Tests

Test same logic with different inputs:

```cpp
class DistanceTest : public ::testing::TestWithParam<
    std::tuple<spanny::position_t, spanny::position_t, double>> {
};

TEST_P(DistanceTest, CalculatesCorrectly) {
  auto [p1, p2, expected] = GetParam();
  auto actual = spanny::distance_between(p1, p2);
  EXPECT_NEAR(actual, expected, 0.01);
}

INSTANTIATE_TEST_SUITE_P(
    VariousPoints,
    DistanceTest,
    ::testing::Values(
        std::make_tuple(spanny::position_t{0, 0}, spanny::position_t{0, 0}, 0.0),
        std::make_tuple(spanny::position_t{0, 0}, spanny::position_t{3, 4}, 5.0),
        std::make_tuple(spanny::position_t{1, 1}, spanny::position_t{4, 5}, 5.0)
    )
);
```

## Debugging Tests

### Running Under Debugger

```bash
# Linux
pixi shell
gdb build/test_rrt
(gdb) run --gtest_filter=MyTest.*
(gdb) break spanny::rrt_t::operator()
(gdb) continue

# macOS
pixi shell
lldb build/test_rrt
(lldb) run --gtest_filter=MyTest.*
(lldb) breakpoint set --name operator()
(lldb) continue
```

### Print Debugging

Add stream operators for custom types:

```cpp
std::ostream& operator<<(std::ostream& os, spanny::position_t const& pos) {
  os << "(" << pos.x << ", " << pos.y << ")";
  return os;
}

// Now positions print nicely in test failures
EXPECT_EQ(actual_pos, expected_pos);  // Shows (1.0, 2.0) vs (3.0, 4.0)
```

### Test Output Debugging

Add diagnostic output:

```cpp
TEST(Planning, DebugOutput) {
  auto result = planner(start, goal, context);

  if (!result.has_value()) {
    std::cout << "Planning failed: " << result.error() << "\n";
  } else {
    std::cout << "Tree size: " << result.value().nodes.size() << "\n";
  }

  ASSERT_TRUE(result.has_value());
}
```

## Best Practices

1. **Use mocks for randomness**: Make stochastic tests deterministic
2. **Test edge cases**: Empty inputs, same start/goal, boundaries
3. **Prefer EXPECT over ASSERT**: Let tests continue to find multiple issues
4. **Use ASSERT for preconditions**: Stop test if setup fails
5. **Write clear test names**: Describe what behavior is being tested
6. **Use fixtures for shared setup**: Reduce duplication
7. **Test error paths**: Verify `std::expected` error cases
8. **Keep tests focused**: One behavior per test case
9. **Use custom matchers**: Make assertions readable
10. **Run coverage regularly**: Ensure tests exercise all code paths

## See Also

- [API: Testing Utilities](../api/testing-utilities.md) - Testing helper functions
- [Pixi Workflow](pixi-workflow.md) - Advanced Pixi commands
- [Contributing Guide](contributing.md) - Code quality standards
- [GoogleTest Primer](https://google.github.io/googletest/primer.html) - GoogleTest documentation
- [GoogleMock for Dummies](https://google.github.io/googletest/gmock_for_dummies.html) - GoogleMock guide
