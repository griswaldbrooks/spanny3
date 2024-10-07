#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <expected>
#include <fstream>
#include <iostream>
#include <mdspan>
#include <optional>
#include <print>
#include <random>
#include <ranges>
#include <span>
#include <string>
#include <vector>

/**
 * @brief Perform a Bernoulli trial with a specified probability.
 * https://en.wikipedia.org/wiki/Bernoulli_trial
 * @param random_generator to use for the trial
 * @param probability of success for the trial (between 0.0 and 1.0)
 * @returns `true` with probability @p probability, `false` otherwise
 */
bool bernoulli_trial(std::uniform_random_bit_generator auto& random_generator, double probability) {
  std::bernoulli_distribution distribution{probability};
  return distribution(random_generator);
}

/**
 * @brief Represents types that behave like a point with x and y coordinates.
 * 
 * @tparam T is the type to check for point-like properties
 */
template <typename T>
concept point_like = requires(T t) {
  { t.x } -> std::convertible_to<double>;
  { t.y } -> std::convertible_to<double>;
};

/**
 * @brief Represents a circle in 2D space.
 */
struct circle_t {
  double x; /**< x coordinate of the center */
  double y; /**< y coordinate of the center */
  double radius; /**< radius of the circle */
};

/**
 * @brief Represents a numerical range with minimum and maximum values.
 */
struct bounds_t {
  double min; /**< Minimum value of the range */
  double max; /**< Maximum value of the range */
};
/**
 * @brief Parameters for the planning algorithm.
 */
struct planning_context_t {
  bounds_t x_limits; /**< minimum and maximum x-values of the planning area */
  bounds_t y_limits; /**< minimum and maximum y-values of the planning area */
  std::size_t expansion_limit; /**< maximum number of nodes to expand during planning */
  double sample_distance; /**< fixed distance to project when extending the tree */  
  double goal_probability; /**< probability of sampling the goal during planning */
  std::vector<circle_t> obstacles; /** obstacles to avoid in planning scene */
};

/**
 * @brief Type alias for node identifiers in the planning tree.
 */
using node_id_t = std::size_t;

/**
 * @brief Represents a displacement in 2D space.
 */
struct displacement_t {
  double x; /**< displacement along x-axis */
  double y; /**< displacement along y-axis */
  // ordering comparators probably don't make sense?
  // auto operator<=>(displacement_t const&) const = default;
};

displacement_t operator+(displacement_t const& lhs, displacement_t const& rhs);
displacement_t operator-(displacement_t const& lhs, displacement_t const& rhs);
displacement_t operator-(displacement_t const& value);
displacement_t operator*(displacement_t const& disp, std::floating_point auto scalar);
displacement_t operator*(std::floating_point auto scalar, displacement_t const& disp);
displacement_t operator/(displacement_t const& disp, std::floating_point auto scalar);
displacement_t& operator+=(displacement_t& lhs, displacement_t const& rhs);
displacement_t& operator-=(displacement_t& lhs, displacement_t const& rhs);
displacement_t& operator*=(displacement_t& disp, std::floating_point auto scalar);
displacement_t& operator/=(displacement_t& disp, std::floating_point auto scalar);

struct position_t {
  double x, y;
};

displacement_t operator-(position_t const& lhs, position_t const& rhs);
position_t operator-(position_t const& pos, displacement_t const& disp);
position_t operator+(position_t const& pos, displacement_t const& disp);
position_t operator+(displacement_t const& disp, position_t const& pos);

template <>
struct std::hash<position_t> {
  std::size_t operator()(position_t const& point) const noexcept {
    auto const h1 = std::hash<double>{}(point.x);
    auto const h2 = std::hash<double>{}(point.y);
    return h1 ^ (h2 << 1);
  }
};

struct node_t {
  explicit node_t(position_t p);
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

auto magnitude(displacement_t const& p);

displacement_t normalize(displacement_t const& d);

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

bool is_between(auto const& value, auto const& lo, auto const& hi)
  requires std::totally_ordered_with<decltype(lo), decltype(value)> and
           std::totally_ordered_with<decltype(value), decltype(hi)>
{
  return lo <= value && value <= hi;
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
                  std::span<circle_t const> obstacles) {
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

bool in_collision(point_like auto const& position, std::span<circle_t const> obstacles) {
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
                                                 std::span<node_t const> nodes) ;

node_t project_sample(planning_context_t const& context, node_t const& sampled,
                      node_t const& closest) ;

std::expected<node_id_t, std::string> expand_tree(planning_context_t const& context,
                                                  node_t sampled_node, tree_t& tree) ;

struct rrt_t {
  explicit rrt_t(uint32_t seed);

  [[nodiscard]] std::expected<tree_t, std::string> operator()(node_t const& start,
                                                              node_t const& goal,
                                                              planning_context_t const& context) ;
 private:
  std::mt19937 random_generator_;
};

