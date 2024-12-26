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

TEST(bresenham_conversion, overloaded_function_check) {
  // GIVEN an rrt
  mock_random_t rng;
  ON_CALL(rng, real_between).WillByDefault([](auto, auto){ return 0.;});
  auto rrt = stochastic::rrt_t{rng};
  // no obstacles, don't sample the goal
  // maybe not sampling the goal means it will fail
  // having a goal so close with a similar sample distance probably means overshoot
  planning_context_t context{.x_limits = {-10., 10.},
                             .y_limits = {-10., 10.},
                             .expansion_limit = 1000,
                             .sample_distance = 1.,
                             .goal_probability = 0.,
                             .obstacles = {}};
  auto const start = position_t{0., 0.};
  auto const goal = position_t{1., 1.};
  // WHEN the pixels are produced via the two bresenham functions
  auto const tree_maybe = rrt(start, goal, context);
  // THEN the two vectors should be the same size and be equal to each other
  EXPECT_TRUE(tree_maybe.has_value()) << tree_maybe.error();
}
}//namespace spanny
