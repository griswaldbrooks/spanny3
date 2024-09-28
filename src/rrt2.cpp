#include <random>
#include <array>
#include <iostream>
#include <vector>
#include "expected.hpp"
#include <string>
#include <cmath>
#include <concepts>
#include <ranges>
#include <algorithm>

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

struct node_t {
  node_id_t id;
  double x, y;
  double cost;
};

struct position_t {
  double x, y;
};

struct edge_t {
  node_id_t parent, child;
  double cost;
};

struct tree_t {
  std::vector<node_t> nodes;
  std::vector<edge_t> edges;
};

auto distance_between(point_like auto const& pose1, point_like auto const& pose2) {
	return std::hypot(pose2.x - pose1.x, pose2.y - pose1.y);
}

auto distance_squared(point_like auto const& pose1, point_like auto const& pose2) {
	auto const dx = pose2.x - pose1.x;
  auto const dy = pose2.y - pose1.y;
  return dx*dx + dy*dy;
}

bool is_between(auto const& value, auto const& lo, auto const& hi)
  requires std::totally_ordered_with<decltype(lo), decltype(value)> and 
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
	return distance_between(node1, node2);
}

/**
 * @brief Parametrically check for if there are collisions between the obstacle and path between nodes
 *
 * @param	node1 first node to check between for obstacles
 * @param	node2 second node to check between for obstacles
 * @param	obstacles to check for collisions with along the line
 * @returns true if along the line between the two nodes it intersects with any obstacle
 */
bool collision_check(node_t const& node1, node_t const& node2, std::vector<circle_t> const& obstacles) {
	auto const A = distance_squared(node1, node2);
	for (auto const& obstacle : obstacles){
		if (distance_between(node1, obstacle) <= obstacle.radius) {
			return true;
    }
		if (distance_between(node2, obstacle) <= obstacle.radius) {
			return true;
    }
		// Try and solve for quadratic equation that results from finding the intersection between a line and circle
		auto const B = 2*((node2.x - node1.x)*(node1.x - obstacle.x) + (node2.y - node1.y)*(node1.y - obstacle.y));
		auto const C = distance_squared(node1, obstacle) - obstacle.radius * obstacle.radius;
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
node_t new_node_along_line(node_id_t node_number, node_t const& node1, node_t const& node2, double distance) {
  // Create a vector in the direction of the second node of magnitude distance
	auto const theta = std::atan2(node2.y - node1.y, node2.x - node1.x);
	auto const x = distance * std::cos(theta) + node1.x;
  auto const y = distance * std::sin(theta) + node1.y;

	return {node_number, x, y, 0.};
}

// 	"""
// 	Generates RRT nodes and edges 
//
// 	:param      x_limits:    The x limits
// 	:param      y_limits:    The y limits
// 	:param      start_node:  The start node
// 	:param      goal_node:   The goal node
// 	:param      obstacles:   The obstacles
// 	"""
tl::expected<tree_t, std::string> rrt(
  bounds_t const& x_lims,
  bounds_t const& y_lims,
  node_t const& start,
  node_t const& goal,
  std::vector<circle_t> const& obstacles
)
{
  // Append the start node to the list of nodes
  std::vector<node_t> nodes;
	nodes.push_back(start);

  // Set max size of RRT to 1000 nodes
	auto const max_size = 1000;
  std::vector<edge_t> edges;
	bool sampled_goal = false;

  // pretty sure this should be held as a member
  std::random_device rd;  // a seed source for the random number engine
  std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> x_distribution(x_lims.min, x_lims.max);
  std::uniform_real_distribution<> y_distribution(y_lims.min, y_lims.max);
  std::uniform_real_distribution<> sample_goal(0., 10.);
  // Start from node 3 (because node 1 is start and node 2 is goal)
	node_id_t running_node_number = 3u;
	while (nodes.size() < max_size) {
  // Sample from a uniform distribution
    auto x_samp = x_distribution(gen);
    auto y_samp = y_distribution(gen);

	  // Should we sample the goal? (Happens 10% of the time)
		if (sample_goal(gen) < 1) {
			x_samp = goal.x;
			y_samp = goal.y;
			sampled_goal = true;
    }

		// Check that the sample is not in an obstacle
		for (auto const& obstacle: obstacles) {
      if (distance_between(obstacle, position_t{x_samp, y_samp}) < obstacle.radius) {
        continue;
      }
    }

		// Find the nearest node
		auto running_min_cost = std::numeric_limits<double>::max();
		node_id_t running_closest_node_idx = 0;
		node_id_t const sampled_node_number = [&] () -> node_id_t {
  		if (sampled_goal)	return 2u; // If sampled goal, the node number is 2
			return running_node_number;
    }();
		auto sampled_node = node_t{sampled_node_number, x_samp, y_samp, 0.};
		for (auto const& node: nodes) {
			auto const cost = heuristic(node, sampled_node);
			if (cost < running_min_cost) {
				running_min_cost = cost;
				running_closest_node_idx = node.id;
      }
    }

		// If the sampled node is too far from the nearest node,
    // make a closer node in the direction of sampled node
		auto const distance = 0.1;
		// If we sampled the goal and the nearest node is too far, continue
		if (sampled_goal and (running_min_cost > distance)) {
			sampled_goal = false;
			continue;
    }
		if (running_min_cost > distance) {
			sampled_node = new_node_along_line(sampled_node_number, nodes.at(running_closest_node_idx), 
                                      sampled_node, distance);
    }
		// Is closest node to current sample collision free?
		if (not collision_check(nodes[running_closest_node_idx], sampled_node, obstacles)){
		  // Get cost to goal
			auto const cost_to_goal = heuristic(nodes.at(running_closest_node_idx), sampled_node);
			// Add goal or current sample to nodes
			if (not sampled_goal) {
				nodes.emplace_back(sampled_node.id, sampled_node.x, sampled_node.y, cost_to_goal);
      }
			else {
				nodes.push_back(goal);
      }
			// Add edge 
			edges.emplace_back(sampled_node_number, nodes.at(running_closest_node_idx).id, running_min_cost);
			running_node_number += 1;
			if (sampled_goal) {
				break;
      }
    }
		else {
			sampled_goal = false;
			continue;
    }
  }

	if (not sampled_goal) {
    return tl::unexpected(std::string{"RRT generation failure"});
  }

	// Sort the nodes by node number
  std::ranges::sort(nodes, {}, &node_t::id);
	return tree_t{nodes, edges};
}

int main() {
	// Define parameters
	auto const x_lims = bounds_t{-0.5, 0.5};
	auto const y_lims = bounds_t{-0.5, 0.5};
	auto const start_node = node_t{1, -0.5, -0.5, 1.4142};
  auto const goal_node = node_t{2, 0.5, 0.5, 0.0};
  auto const obstacles = std::vector<circle_t>{
    {0.0, 0.0, 0.2},//
    {0.0, 0.1, 0.2},//
    {0.3, 0.2, 0.2},//
    {-0.3, -0.2, 0.2},//
    {-0.1, -0.4, 0.2},//
    {-0.2, 0.3, 0.2},//
    {0.3, -0.3, 0.2},//
    {0.1, 0.4, 0.2}//
    };

	// Try to generate the RRT until success
	int count = 0;
	while(count < 5){
		auto const tree_maybe = rrt(x_lims, y_lims, start_node, goal_node, obstacles);
		if(tree_maybe.has_value()) {
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

