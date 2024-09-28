#include <algorithm>
#include <vector>
#include <map>
#include <stdexcept>
#include <random>
#include "mdspan.hpp"
#include "expected.hpp"
#include <utility>
#include <iostream>

struct position_t {
  double x, y;
};

template<>
struct std::hash<position_t>
{
    std::size_t operator()(position_t const& p) const noexcept
    {
        std::size_t h1 = std::hash<double>{}(p.x);
        std::size_t h2 = std::hash<double>{}(p.y);
        return h1 ^ (h2 << 1);
    }
};
using node_id_t = std::size_t;

struct tree_t {
  std::map<node_id_t, position_t> nodes;
  std::multimap<node_id_t, node_id_t> edges;
};

using grid_t = std::mdspan<unsigned char, std::dextents<std::size_t, 2>>;

tl::expected<node_id_t, std::string> find_nearest_neighbor(std::map<node_id_t, position_t> const& nodes, position_t const& target) {
    if (nodes.empty()) {
        return tl::unexpected(std::string{"The tree contains no nodes."});
    }

    // Create a vector to store pairs of (distance_squared, node_id_t)
    std::vector<std::pair<double, node_id_t>> distances;

    // Transform the map into a vector of distances
    std::ranges::transform(nodes, std::back_inserter(distances),
        [&target](auto const& pair) -> std::pair<double, node_id_t> {
            auto const& [node_id, node] = pair;
            double const dx = node.x - target.x;
            double const dy = node.y - target.y;
            double const distance_squared = dx * dx + dy * dy;
            return {distance_squared, node_id};
        });

    // Find the minimum element based on distance
    auto const min_it = std::ranges::min_element(distances,
        [](const auto& lhs, const auto& rhs) {
            return lhs.first < rhs.first; // Compare based on distance_squared
        });

    return min_it->second; // Return the node_id_t
}

template <typename T>
bool in_bounds(T const& value, T const& lo, T const& hi) {
  return lo <= value && value <= hi;
}

template <typename T>
bool in_bounds(T const& value, std::dextents<std::size_t, 2> bounds) {
  return in_bounds(value.x, 0, static_cast<decltype(T::x)>(bounds.extent(1))) &&
         in_bounds(value.y, 0, static_cast<decltype(T::y)>(bounds.extent(0)));
}

tl::expected<bool, std::string> check_collision(position_t const& node1, position_t const& node2, grid_t map) {
  // check that nodes are in bounds of map 
  if (!in_bounds(node1, map.extents()) or !in_bounds(node2, map.extents()) ) {
    return tl::unexpected(std::string{"nodes are outside of the map"});
  }
  // linearly interpolate between nodes
  // at each point in between, check if the pixel in the map is occupied
  return false;
}

position_t random_point(std::dextents<std::size_t, 2> shape, position_t const& goal, double goal_bias) {
  if (rand()/(double)RAND_MAX < goal_bias) {
    return goal;
  }
  // pretty sure this should be held as a member
  std::random_device rd;  // a seed source for the random number engine
  std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<std::size_t> x_distribution(0, shape.extent(1));
  std::uniform_int_distribution<std::size_t> y_distribution(0, shape.extent(0));
  auto const new_x = static_cast<double>(x_distribution(gen));
  auto const new_y = static_cast<double>(y_distribution(gen));
  std::cout << new_x << ", " << new_y << std::endl;
  return {new_x, new_y};
}

void plan_rrt(position_t const& start, position_t const& goal, grid_t map, int n_points) {
  int j = 0;
  bool goal_found = false;
  int step_size = 5;

  tree_t tree;
  tree.nodes.emplace(std::hash<position_t>{}(start), start);

  auto make_random_point = [=]{
    return random_point(map.extents(), goal, 10.);
  };

  while (j < n_points) {
    auto const rand_point = make_random_point();
    auto const near_node_maybe = find_nearest_neighbor(tree.nodes, rand_point);
    if (!near_node_maybe.has_value()) {
      std::cout << near_node_maybe.error() << std::endl;
      return;
    }
    auto const& near_node = tree.nodes.at(near_node_maybe.value());

    // get point on line with step, at set distance
    auto const dx = rand_point.x - near_node.x; // Updated from row to x 
    auto const dy = rand_point.y - near_node.y; // Updated from col to y 
    auto const ang = std::atan2(dy, dx);
    auto const step_x = near_node.x + step_size * std::cos(ang);
    auto const step_y = near_node.y + step_size * std::sin(ang);
    auto const step = position_t{step_x, step_y};

    if (auto const collision_maybe = check_collision(step, near_node);
      !collision_maybe.has_value() or collision_maybe.value()) {
      // There was a collision, pass for now, but really we should
      // just try again with another point 
      ++j;
      continue;
    }
    // if no collision check if goal state has been reached
    if (goal.x == step.x and goal.y == step.y) { // Updated to x and y
      goal.parent = &near_node;
      verticies.push_back(goal);
      goal_found = true;
      break;
    }
    // if goal state has not been reached, add node to Tree
    step.parent = &near_node;
    //step.cost = dis(step, near_node) + step.parent->cost;
    verticies.push_back(step);

    // if goal is in vicinity of new node, directly connect the goal node 
    double dis_goal = dis(goal, step);
    if (dis_goal < 10. and check_collision(step, goal)) {
      goal.parent = &near_node;
      //goal.cost = dis(step, near_node) + goal.parent->cost;
      verticies.push_back(goal);
      goal_found = true;
      break;
    }
  }
}

int main() {
  return 0;
}

