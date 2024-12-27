// C++ Standard Library

// Gtest
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// Code to test
#include "spanny/rrt.hpp"
namespace spanny {

struct mock_random_t {
  MOCK_METHOD(double, real_between, (double min, double max));
  MOCK_METHOD(bool, yes_maybe, (double probability));
};

TEST(TreeGeneration, BadRandom) {
  // GIVEN a faulty random number generator
  mock_random_t rng;
  ON_CALL(rng, real_between).WillByDefault([](auto, auto) { return 0.; });
  // WHEN the rrt uses it to plan
  auto make_random_tree = stochastic::rrt_t{rng};
  planning_context_t context{.x_limits = {-10., 10.},
                             .y_limits = {-10., 10.},
                             .expansion_limit = 1000,
                             .sample_distance = 1.,
                             .goal_probability = 0.,
                             .obstacles = {}};
  auto const start = position_t{0., 0.};
  auto const goal = position_t{1., 1.};
  auto const tree_maybe = make_random_tree(start, goal, context);
  // THEN it should fail to reach the goal
  EXPECT_FALSE(tree_maybe.has_value()) << tree_maybe.error();
}
}  // namespace spanny
