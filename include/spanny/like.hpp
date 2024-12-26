#pragma once
#include <concepts>

namespace spanny::like {
template <typename T>
concept some_random_generator = requires(T t, double probability, double min, double max) {
  { t.real_between(min, max) } -> std::convertible_to<double>;
  { t.yes_maybe(probability) } -> std::convertible_to<bool>;
};
  
/**
 * @brief Represents types that behave like a point with x and y coordinates.
 *
 * @tparam T is the type to check for point-like properties
 */
template <typename T>
concept some_point = requires(T t) {
  { t.x } -> std::convertible_to<double>;
  { t.y } -> std::convertible_to<double>;
};

}
