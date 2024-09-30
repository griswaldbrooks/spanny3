#include "expected.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>
#include <random>
#include <ranges>
#include <string>
#include <vector>

template <typename T>
concept point_like = requires(T t) {
  { t.x } -> std::convertible_to<double>;
  { t.y } -> std::convertible_to<double>;
};

struct circle_t {
  double x, y, radius;
};

struct bounds_t {
  double min, max;
};

using node_id_t = std::size_t;

struct position_t {
  double x, y;
};

template <>
struct std::hash<position_t> {
  std::size_t operator()(position_t const& point) const noexcept {
    auto const h1 = std::hash<double>{}(point.x);
    auto const h2 = std::hash<double>{}(point.y);
    return h1 ^ (h2 << 1);
  }
};

struct node_t {
  node_id_t id;
  position_t position;
  double cost;
};

struct edge_t {
  node_id_t parent, child;
  double cost;
};

auto distance_between(point_like auto const& pose1, point_like auto const& pose2) {
  return std::hypot(pose2.x - pose1.x, pose2.y - pose1.y);
}

auto distance_squared(point_like auto const& pose1, point_like auto const& pose2) {
  auto const dx = pose2.x - pose1.x;
  auto const dy = pose2.y - pose1.y;
  return dx * dx + dy * dy;
}

bool is_between(auto const& value, auto const& lo, auto const& hi) requires
    std::totally_ordered_with<decltype(lo), decltype(value)> and
    std::totally_ordered_with<decltype(value), decltype(hi)> {
  return lo <= value && value <= hi;
}

/**
 * @brief Calculates the to-go cost
 * @param	node1 to estimate distance from
 * @param	node2 to estimate distance to
 * @returns the distance between nodes
 */
double heuristic(node_t const& node1, node_t const& node2) {
  return distance_between(node1.position, node2.position);
}

/**
 * @brief Parametrically check for if there are collisions between the obstacle and path between
 * nodes
 *
 * @param	node1 first node to check between for obstacles
 * @param	node2 second node to check between for obstacles
 * @param	obstacles to check for collisions with along the line
 * @returns true if along the line between the two nodes it intersects with any obstacle
 */
bool in_collision(node_t const& node1, node_t const& node2,
                  std::vector<circle_t> const& obstacles) {
  auto const A = distance_squared(node1.position, node2.position);
  for (auto const& obstacle : obstacles) {
    if (distance_between(node1.position, obstacle) <= obstacle.radius) {
      return true;
    }
    if (distance_between(node2.position, obstacle) <= obstacle.radius) {
      return true;
    }
    // Try and solve for quadratic equation that results from finding the intersection between a
    // line and circle
    auto const B = 2 * ((node2.position.x - node1.position.x) * (node1.position.x - obstacle.x) +
                        (node2.position.y - node1.position.y) * (node1.position.y - obstacle.y));
    auto const C = distance_squared(node1.position, obstacle) - obstacle.radius * obstacle.radius;
    // If the discriminant is zero or positive, then check if the line segment intersects
    // as opposed to the infinite line
    if (auto const discriminant = B * B - 4 * A * C; discriminant >= 0.) {
      auto const sqrt_d = std::sqrt(discriminant);
      auto const t1 = (-B + sqrt_d) / (2 * A);
      auto const t2 = (-B - sqrt_d) / (2 * A);
      if (is_between(t1, 0., 1.) or is_between(t2, 0., 1.)) {
        return true;
      }
    }
    // If the discriminant is less zero, then the line does not intersect with the circle
  }
  return false;
}

/**
 * @brief Create a new node along a line from node1 to node2, given the distance from node 1
 *
 * @param node_number to id the new node with
 * @param node1 to use at starting point
 * @param node2 to go in the direction of
 * @param distance from node1 to place the new node
 * @returns a new node along the line
 */
node_t new_node_along_line(node_t const& node1, node_t const& node2, double distance) {
  // Create a vector in the direction of the second node of magnitude distance
  auto const theta =
      std::atan2(node2.position.y - node1.position.y, node2.position.x - node1.position.x);
  auto const x = distance * std::cos(theta) + node1.position.x;
  auto const y = distance * std::sin(theta) + node1.position.y;
  auto const p = position_t{x, y};

  return {std::hash<position_t>{}(p), p, 0.};
}

bool in_collision(point_like auto const& position, std::vector<circle_t> const& obstacles) {
  for (auto const& obstacle : obstacles) {
    if (distance_between(obstacle, position) < obstacle.radius) {
      return true;
    }
  }
  return false;
}

struct tree_t {
  // TODO: add ability to be deterministically seeded
  tree_t(bounds_t const& x_lims, bounds_t const& y_lims, std::vector<circle_t> obstacles)
      : sample_goal_distribution_{0., 1.},
        x_distribution_{x_lims.min, x_lims.max},
        y_distribution_{y_lims.min, y_lims.max},
        obstacles_{std::move(obstacles)} {}

  // 	"""
  // 	Generates RRT nodes and edges
  //
  // 	:param      x_limits:    The x limits
  // 	:param      y_limits:    The y limits
  // 	:param      start_node:  The start node
  // 	:param      goal_node:   The goal node
  // 	:param      obstacles:   The obstacles
  // 	"""
  tl::expected<bool, std::string> operator()(node_t const& start,
                                             [[maybe_unused]] node_t const& goal) {
    // Append the start node to the list of nodes
    nodes_.push_back(start);

    // Set max size of RRT to 1000 nodes
    auto const max_size = 1000;
    bool sampled_goal = false;

    while (nodes_.size() < max_size) {
      if (sample_goal()) {
        auto const closest_node =
            std::ranges::min_element(nodes_, [&](auto const& lhs, auto const& rhs) {
              return heuristic(lhs, goal) < heuristic(rhs, goal);
            });
        auto const distance = 0.1;
        // If we sampled the goal and the nearest node is too far, continue
        if (heuristic(*closest_node, goal) > distance) {
          continue;
        }
        nodes_.push_back(goal);
        // Add edge
        edges_.emplace_back(goal.id, closest_node->id, heuristic(*closest_node, goal));
        sampled_goal = true;
        break;
      } else {
        expand_random();
      }
    }

    std::cout << "nodes_.size() = " << nodes_.size() << "\n";
    for (auto const& node : nodes_) {
      std::cout << node.position.x << ", " << node.position.y << "\n";
    }
    if (not sampled_goal) {
      return tl::unexpected(std::string{"RRT generation failure"});
    }

    // Sort the nodes by node number
    std::ranges::sort(nodes_, {}, &node_t::id);
    // return tree;
    return true;
  }

 private:
  bool sample_goal(double goal_probability = 0.1) {
    if (sample_goal_distribution_(gen_) < goal_probability) {
      return true;
    }
    return false;
  }

  bool expand_random() {
    auto const sampled_node_maybe = sample_random_node();
    if (not sampled_node_maybe.has_value()) {
      return false;
    }
    auto sampled_node = sampled_node_maybe.value();
    // Find the nearest node
    auto const closest_node =
        std::ranges::min_element(nodes_, [&](auto const& lhs, auto const& rhs) {
          return heuristic(lhs, sampled_node) < heuristic(rhs, sampled_node);
        });
    // If the sampled node is too far from the nearest node,
    // make a closer node in the direction of sampled node
    auto const distance = 0.1;
    // If we sampled the goal and the nearest node is too far, continue
    // if (sampled_goal and (running_min_cost > distance)) {
    // 	sampled_goal = false;
    // 	continue;
    //   }
    if (heuristic(*closest_node, sampled_node) > distance) {
      sampled_node = new_node_along_line(*closest_node, sampled_node, distance);
    }
    // Is closest node to current sample collision free?
    if (not in_collision(*closest_node, sampled_node, obstacles_)) {
      // Get cost to goal
      auto const cost_to_goal = heuristic(*closest_node, sampled_node);
      // Add goal or current sample to nodes
      // if (not sampled_goal) {
      nodes_.emplace_back(sampled_node.id, sampled_node.position, cost_to_goal);
      // } else {
      // nodes_.push_back(goal);
      // }
      // Add edge
      edges_.emplace_back(sampled_node.id, closest_node->id,
                          heuristic(*closest_node, sampled_node));
      // if (sampled_goal) {
      // break;
      // }
      return true;
    }
    return false;
  }

  tl::expected<node_t, std::string> sample_random_node() {
    // Sample from a uniform distribution
    auto const sample_position = position_t{x_distribution_(gen_), y_distribution_(gen_)};
    // std::cout << (x_samp > 0 ? "+" : "-") << (y_samp > 0 ? "+" : "-") << ",";
    // std::cout << sample_position.x << ", " << sample_position.y << "\n";

    // Check that the sample is not in an obstacle
    if (in_collision(sample_position, obstacles_)) {
      return tl::unexpected(std::string{"Sampled node is in collision with obstacle"});
    }
    return node_t{std::hash<position_t>{}(sample_position), sample_position, 0.};
  }

  std::mt19937 gen_;
  std::uniform_real_distribution<> sample_goal_distribution_;
  std::uniform_real_distribution<> x_distribution_;
  std::uniform_real_distribution<> y_distribution_;
  std::vector<node_t> nodes_;
  std::vector<edge_t> edges_;
  std::vector<circle_t> obstacles_;
};

int main() {
  // Define parameters
  auto const x_lims = bounds_t{-0.5, 0.5};
  auto const y_lims = bounds_t{-0.5, 0.5};
  auto const start_node = node_t{0, -0.4, -0.4, 1.4142};
  auto const goal_node = node_t{1, 0.4, 0.4, 0.0};
  auto const obstacles = std::vector<circle_t>{
      {0.0, 0.0, 0.1},    //
      {0.0, 0.1, 0.1},    //
      {0.3, 0.2, 0.1},    //
      {-0.3, -0.2, 0.1},  //
      {-0.1, -0.4, 0.1},  //
      {-0.2, 0.3, 0.1},   //
      {0.3, -0.3, 0.1},   //
      {0.1, 0.4, 0.1}     //
  };
  auto const obstacles2 = std::vector<circle_t>{{0.2, 0.2, 0.1}};
  // Try to generate the RRT until success
  auto rrt = tree_t(x_lims, y_lims, obstacles2);
  int count = 0;
  while (count < 1) {
    auto const tree_maybe = rrt(start_node, goal_node);
    if (tree_maybe.has_value()) {
      std::cout << "Something worked?" << std::endl;
      break;
    }
    std::cout << "Retrying...\n";
    ++count;
  }
  std::cout << count << std::endl;
  // path = None
  // if reached_goal_flag:
  // 	print("Generating path")
  // 	path = PathGenerator(nodes, edges, start_node[0], goal_node[0])

  return 0;
}
