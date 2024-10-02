#include <expected>
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>
#include <optional>
#include <random>
#include <ranges>
#include <string>
#include <vector>
#include "json.hpp"

using json = nlohmann::json;
bool bernoulli_trial(std::uniform_random_bit_generator auto& random_generator, double probability) {
  std::bernoulli_distribution distribution{probability};
  return distribution(random_generator);
}
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

struct planning_context_t {
  bounds_t x_limits, y_limits;
  std::size_t expansion_limit;
  double sample_distance;
  double goal_probability;
  std::vector<circle_t> obstacles;
};

using node_id_t = std::size_t;

struct displacement_t {
  double x, y;
  // ordering comparators probably don't make sense?
  // auto operator<=>(displacement_t const&) const = default;
};

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

struct position_t {
  double x, y;
};

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

template <>
struct std::hash<position_t> {
  std::size_t operator()(position_t const& point) const noexcept {
    auto const h1 = std::hash<double>{}(point.x);
    auto const h2 = std::hash<double>{}(point.y);
    return h1 ^ (h2 << 1);
  }
};

struct node_t {
  explicit node_t(position_t p) : id{std::hash<position_t>{}(p)}, position{std::move(p)} {}
  node_id_t id;
  position_t position;
};

struct edge_t {
  node_id_t parent, child;
  double cost;
};

struct tree_t {
  std::vector<node_t> nodes;
  std::vector<edge_t> edges;
};

auto magnitude(displacement_t const& p) { return std::hypot(p.x, p.y); }

displacement_t normalize(displacement_t const& d) {
  auto const l = magnitude(d);
  return d / l;
}

auto distance_between(point_like auto const& pose1, point_like auto const& pose2) {
  return std::hypot(pose2.x - pose1.x, pose2.y - pose1.y);
}

auto distance_squared(point_like auto const& pose1, point_like auto const& pose2) {
  auto const dx = pose2.x - pose1.x;
  auto const dy = pose2.y - pose1.y;
  return dx * dx + dy * dy;
}

/**
 * @brief Project a point from a start in a direction for a distance.
 *
 * @param origin is the starting point for the projection
 * @param target to project towards
 * @param distance from origin to project
 * @returns a new point, @p distance away from @p origin in the direction of @p target
 */
auto project(point_like auto const& origin, point_like auto const& target, double distance) {
  auto const direction = normalize(target - origin);
  return origin + distance * direction;
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
 * @brief Check for collisions between the obstacles and the line between points
 *
 * @param	p1 is the start of the line segment
 * @param	p2 is the end of the line segment
 * @param	obstacles to check for collisions with along the line
 * @returns `true` if there is an intersection along the line with an obstacle
 */
bool in_collision(point_like auto const& p1, point_like auto const& p2,
                  std::vector<circle_t> const& obstacles) {
  auto const A = distance_squared(p1, p2);
  return std::ranges::any_of(obstacles, [&](auto const& obstacle) {
    if (distance_between(p1, obstacle) <= obstacle.radius) {
      return true;
    }
    if (distance_between(p2, obstacle) <= obstacle.radius) {
      return true;
    }
    // Try and solve for quadratic equation that results from finding the intersection between a
    // line and circle
    auto const B = 2 * ((p2.x - p1.x) * (p1.x - obstacle.x) + (p2.y - p1.y) * (p1.y - obstacle.y));
    auto const C = distance_squared(p1, obstacle) - obstacle.radius * obstacle.radius;
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
  return false;
  });
}

bool in_collision(point_like auto const& position, std::vector<circle_t> const& obstacles) {
  for (auto const& obstacle : obstacles) {
    if (distance_between(obstacle, position) < obstacle.radius) {
      return true;
    }
  }
  return false;
}

std::expected<node_t, std::string> sample_space(
    std::uniform_random_bit_generator auto& random_generator, planning_context_t const& context) {
  auto x_distribution =
      std::uniform_real_distribution<>{context.x_limits.min, context.x_limits.max};
  auto y_distribution =
      std::uniform_real_distribution<>{context.y_limits.min, context.y_limits.max};
  auto const sample_position =
      position_t{x_distribution(random_generator), y_distribution(random_generator)};
  if (in_collision(sample_position, context.obstacles)) {
    return std::unexpected(std::string{"Sampled node is in collision with obstacle"});
  }
  return node_t{sample_position};
}

std::expected<node_t, std::string> sample_space_or_goal(
    std::uniform_random_bit_generator auto& random_generator, planning_context_t const& context,
    std::optional<node_t> goal_maybe) {
  if (goal_maybe and bernoulli_trial(random_generator, context.goal_probability)) {
    return goal_maybe.value();
  }
  return sample_space(random_generator, context);
}

std::expected<node_t, std::string> find_neighbor(node_t const& node,
                                                std::vector<node_t> const& nodes) {
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

struct rrt_t {
  explicit rrt_t(uint32_t seed) : random_generator_{seed} {}

  [[nodiscard]] std::expected<tree_t, std::string> operator()(node_t const& start,
                                                             node_t const& goal,
                                                             planning_context_t const& context) {
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

 private:
  std::mt19937 random_generator_;
};

int main() {
  std::ifstream bin_file{"src/spanny3/config/scenario.json"};
  json bin_config = json::parse(bin_file);
  // Define parameters
  auto const x_lims = bounds_t{-0.5, 0.5};
  auto const y_lims = bounds_t{-0.5, 0.5};
  auto const start_node = node_t{position_t{-0.4, -0.4}};
  auto const goal_node = node_t{position_t{0.4, 0.4}};
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
  auto const context = planning_context_t{x_lims, y_lims, 1000, 0.1, 0.1, obstacles2};
  // Try to generate the RRT until success
  auto rrt = rrt_t(std::random_device{}());
  auto const tree_maybe = rrt(start_node, goal_node, context);
  if (tree_maybe.has_value()) {
    std::cout << "Something worked?" << std::endl;
    auto const& tree = tree_maybe.value();
    std::cout << "nodes.size() = " << tree.nodes.size() << "\n";
    for (auto const& node : tree.nodes) {
      std::cout << node.position.x << ", " << node.position.y << "\n";
    }
    break;
  }
  return 0;
}
