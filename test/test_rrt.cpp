// C++ Standard Library

// Gtest
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <format>
#include <ranges>
// Code to test
#include "spanny/rrt.hpp"

std::ostream& operator<<(std::ostream& os, spanny::position_t const& position) {
  os << position.x << ", " << position.y;
  return os;
}

std::ostream& operator<<(std::ostream& os, spanny::node_t const& node) {
  os << "id: " << node.id << "; pos: " << node.position;
  return os;
}

std::ostream& operator<<(std::ostream& os, spanny::edge_t const& edge) {
  os << "parent: " << edge.parent << "; child: " << edge.child << "; cost: " << edge.cost;
  return os;
}

std::ostream& operator<<(std::ostream& os, spanny::tree_t const& tree) {
  os << "Nodes:\n";
  for(auto const& node:tree.nodes) {
    os << node << "\n";
  }
  os << "Edges:\n";
  for(auto const& edge:tree.edges) {
    os << edge << "\n";
  }
  return os;
}
namespace spanny {
/**
* @brief Creates a tree assuming same start and goal
  * @param point used to instantiate the nodes and edges for the tree
  * @returns the tree made from the same start and goal
  */
tree_t make_same_start_goal_tree(position_t const& point) {
  std::vector<node_t> expected_nodes{node_t{point}, node_t{point}};
  std::vector<edge_t> expected_edges{edge_t{expected_nodes.at(0).id, expected_nodes.at(1).id, 0}};
  return tree_t{.nodes=expected_nodes, .edges=expected_edges};
}

/** 
* @brief Matcher function for checking if two trees are the same
* @param tolerance for how far the distance between the expected and given positions is allowed
*/
MATCHER_P(IsSameTree, expected_tree, tolerance, "") { 
  auto const& given_tree = arg;
  for(auto const& [expected_node, given_node] : std::views::zip(expected_tree.nodes, given_tree.nodes)) {
    if(expected_node.id!=given_node.id) {
      return false;
    }
    if(spanny::distance_between(expected_node.position, given_node.position) > tolerance) {
      return false;
    }
  }
  for(auto const& [expected_edge, given_edge] : std::views::zip(expected_tree.edges, given_tree.edges)) {
    if(expected_edge.parent != given_edge.parent) {
      return false;
    }
    if(expected_edge.child != given_edge.child) {
      return false;
    }
    if(expected_edge.cost != given_edge.cost) {
      return false;
    }
  }
  return true;
}	
/**
  * @brief mocks random generation function for `rrt_t`
  */ 
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


TEST(Planning, SameStartEnd) {
  // GIVEN a start and end point that are the same 
  // AND an initialized rrt
  mock_random_t rng;
  ON_CALL(rng, real_between).WillByDefault([](auto, auto) { return 0.; });
  auto make_random_tree = stochastic::rrt_t{rng};
  planning_context_t context{.x_limits = {-10., 10.},
                             .y_limits = {-10., 10.},
                             .expansion_limit = 1000,
                             .sample_distance = 1.,
                             .goal_probability = 0.,
                             .obstacles = {}};

  auto const start = position_t{0., 0.};
  auto const goal = position_t{0., 0.};
  // WHEN planning between those two points 
  auto const tree_maybe = make_random_tree(start, goal, context);
  // THEN the plan should only contain the start and goal
  ASSERT_TRUE(tree_maybe.has_value());
  auto const& tree = tree_maybe.value();
  constexpr auto tolerance = 1e-5;
  EXPECT_THAT(tree, IsSameTree(make_same_start_goal_tree(start), tolerance));
  EXPECT_THAT(tree, IsSameTree(make_same_start_goal_tree(goal), tolerance));
}

}  // namespace spanny
