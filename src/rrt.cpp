#include "rrt.hpp"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <expected>
#include <mdspan>
#include <optional>
#include <random>
#include <ranges>
#include <span>
#include <string>
#include <vector>

displacement_t operator+(displacement_t const& lhs, displacement_t const& rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}
displacement_t operator-(displacement_t const& lhs, displacement_t const& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}
displacement_t operator-(displacement_t const& value) { return {-value.x, -value.y}; }
displacement_t operator*(displacement_t const& disp, std::floating_point auto scalar) {
  return {disp.x * scalar, disp.y * scalar};
}
displacement_t operator*(std::floating_point auto scalar, displacement_t const& disp) {
  return {disp.x * scalar, disp.y * scalar};
}
displacement_t operator/(displacement_t const& disp, std::floating_point auto scalar) {
  return {disp.x / scalar, disp.y / scalar};
}
displacement_t& operator+=(displacement_t& lhs, displacement_t const& rhs) {
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  return lhs;
}
displacement_t& operator-=(displacement_t& lhs, displacement_t const& rhs) {
  lhs.x -= rhs.x;
  lhs.y -= rhs.y;
  return lhs;
}
displacement_t& operator*=(displacement_t& disp, std::floating_point auto scalar) {
  disp.x *= scalar;
  disp.y *= scalar;
  return disp;
}
displacement_t& operator/=(displacement_t& disp, std::floating_point auto scalar) {
  disp.x /= scalar;
  disp.y /= scalar;
  return disp;
}

displacement_t operator-(position_t const& lhs, position_t const& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}
position_t operator-(position_t const& pos, displacement_t const& disp) {
  return {pos.x - disp.x, pos.y - disp.y};
}
position_t operator+(position_t const& pos, displacement_t const& disp) {
  return {pos.x + disp.x, pos.y + disp.y};
}
position_t operator+(displacement_t const& disp, position_t const& pos) {
  return {pos.x + disp.x, pos.y + disp.y};
}

node_t::node_t(position_t p) : id{std::hash<position_t>{}(p)}, position{std::move(p)} {}

auto magnitude(displacement_t const& p) { return std::hypot(p.x, p.y); }

displacement_t normalize(displacement_t const& d) {
  auto const l = magnitude(d);
  return d / l;
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

std::expected<node_t, std::string> find_neighbor(node_t const& node,
                                                 std::span<node_t const> nodes) {
  auto const closest_node = std::ranges::min_element(nodes, [&](auto const& lhs, auto const& rhs) {
    return distance_between(lhs.position, node.position) <
           distance_between(rhs.position, node.position);
  });
  if (closest_node == nodes.end()) {
    return std::unexpected(std::string{"Node does not have any neighbors"});
  }
  return *closest_node;
}

node_t project_sample(planning_context_t const& context, node_t const& sampled,
                      node_t const& closest) {
  // If the sampled node is too far from the nearest node,
  // make a closer node in the direction of sampled node
  if (distance_between(closest.position, sampled.position) > context.sample_distance) {
    return node_t{project(closest.position, sampled.position, context.sample_distance)};
  }
  return sampled;
}

std::expected<node_id_t, std::string> expand_tree(planning_context_t const& context,
                                                  node_t sampled_node, tree_t& tree) {
  return find_neighbor(sampled_node, tree.nodes)
      .and_then([&](auto const& closest) -> std::expected<node_id_t, std::string> {
        auto const sample = project_sample(context, sampled_node, closest);
        if (in_collision(closest.position, sample.position, context.obstacles)) {
          return std::unexpected(std::string{"Sampled node was in collision"});
        }
        // Add to tree
        auto const cost = heuristic(closest, sample);
        tree.nodes.emplace_back(sample.position);
        tree.edges.emplace_back(sample.id, closest.id, cost);
        return sample.id;
      });
}

rrt_t::rrt_t(uint32_t seed) : random_generator_{seed} {}

[[nodiscard]] std::expected<tree_t, std::string> rrt_t::operator()(
    node_t const& start, node_t const& goal, planning_context_t const& context) {
  auto tree = tree_t{};
  tree.nodes.push_back(start);
  auto const sample_the_space = [&] {
    return sample_space_or_goal(random_generator_, context, goal);
  };
  auto const expand_the_tree = [&](auto const& node) { return expand_tree(context, node, tree); };
  while (tree.nodes.size() < context.expansion_limit) {
    auto const id_maybe = sample_the_space().and_then(expand_the_tree);
    if (id_maybe == goal.id) {
      // Sort the nodes by node number, for some reason...
      std::ranges::sort(tree.nodes, {}, &node_t::id);
      return tree;
    }
  }
  return std::unexpected(std::string{"RRT failed to reach goal"});
}
