#include "cxxopts.hpp"
#include "json.hpp"
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

#include "rrt.hpp"

using json = nlohmann::json;

// TODO: Move this to a utility library separate form rrt library
std::tuple<node_t, node_t, planning_context_t> parse(json const& scenario) {
  auto obstacles = std::vector<circle_t>{};
  std::ranges::transform(scenario["obstacles"].get<std::vector<std::vector<double>>>(),
                         std::back_inserter(obstacles),
                         [](auto const& c) { return circle_t{c[0], c[1], c[2]}; });
  auto const context =
      planning_context_t{.x_limits = bounds_t{scenario["x_limits"][0], scenario["x_limits"][1]},
                         .y_limits = bounds_t{scenario["y_limits"][0], scenario["y_limits"][1]},
                         .expansion_limit = scenario["expansion_limit"],
                         .sample_distance = scenario["sample_distance"],
                         .goal_probability = scenario["goal_probability"],
                         .obstacles = obstacles};
  auto const start = node_t{position_t{scenario["start"][0], scenario["start"][1]}};
  auto const goal = node_t{position_t{scenario["goal"][0], scenario["goal"][1]}};
  return {start, goal, context};
}

int main(int argc, char** argv) {
  cxxopts::Options options("rrt", "Plans path through scenario using rrt");
  options.add_options()(
      "s,scenario", "scenario to plan through",
      cxxopts::value<std::string>()->default_value("src/spanny3/config/scenario.json"))  //
      ("h,help", "print usage");

  auto const args = options.parse(argc, argv);
  if (args.count("help")) {
    std::println("{}", options.help());
    return 0;
  }
  auto const filename = args["scenario"].as<std::string>();

  std::ifstream scenario_file{filename};
  json scenario = json::parse(scenario_file);
  auto const [start, goal, context] = parse(scenario);

  auto rrt = rrt_t(std::random_device{}());
  auto const result = rrt(start, goal, context)
                          .transform([](auto const& tree) {
                            std::ranges::for_each(tree.nodes, [](auto const& node) {
                              std::println("{:.3f}, {:.3f}", node.position.x, node.position.y);
                            });
                            return 0;
                          })
                          .transform_error([](auto const& error) {
                            std::println(std::cerr, "{}", error);
                            return 1;
                          });
  return result.value_or(result.error());
}
