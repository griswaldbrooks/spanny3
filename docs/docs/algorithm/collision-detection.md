---
sidebar_position: 2
---

# Collision Detection

Spanny3 implements efficient collision detection for circular obstacles using analytical line-circle intersection tests. This page explains the mathematics and implementation.

## Overview

The planner needs to verify that edges between tree nodes don't intersect with obstacles. Two collision checks are performed:

1. **Point collision**: Is a single position inside an obstacle?
2. **Line segment collision**: Does a line between two positions intersect an obstacle?

## Point-Circle Collision

The simplest check: is a point inside a circle?

### Mathematics

A point **P** is inside circle **C** with center **(cx, cy)** and radius **r** if:

```
distance(P, C) < r
```

More efficiently, using squared distance to avoid `sqrt()`:

```
(P.x - C.x)² + (P.y - C.y)² < r²
```

### Implementation

```cpp
// From include/spanny/rrt.hpp:192-199
bool in_collision(like::some_point auto const& position,
                  std::span<circle_t const> obstacles) {
  for (auto const& obstacle : obstacles) {
    if (distance_between(obstacle, position) < obstacle.radius) {
      return true;
    }
  }
  return false;
}
```

**Complexity**: O(m) where m = number of obstacles

## Line Segment-Circle Collision

The complex case: does a line segment intersect a circle?

### Problem Setup

Given:
- Line segment from **P1** to **P2**
- Circle **C** with center **(cx, cy)** and radius **r**

Find: Does the segment intersect the circle?

### Mathematical Approach

The line segment can be parameterized as:

```
L(t) = P1 + t(P2 - P1),  where 0 ≤ t ≤ 1
```

The segment intersects the circle when:

```
distance(L(t), C) = r
```

This becomes a **quadratic equation** in **t**.

### Derivation

Let **d = P2 - P1** (direction vector). We need:

```
||L(t) - C||² = r²
||(P1 + td) - C||² = r²
```

Expanding:

```
||P1 - C + td||² = r²
(P1 - C)·(P1 - C) + 2t(P1 - C)·d + t²(d·d) = r²
```

This is a quadratic equation **At² + Bt + C = 0** where:

```
A = d·d = ||P2 - P1||²
B = 2(P1 - C)·d
C = ||P1 - C||² - r²
```

### Solution Algorithm

1. **Check endpoints** first (optimization):
   - If P1 or P2 is inside the circle, return collision

2. **Compute discriminant**:
   ```
   Δ = B² - 4AC
   ```

3. **Interpret discriminant**:
   - **Δ < 0**: Line doesn't intersect circle (no real solutions)
   - **Δ = 0**: Line is tangent to circle (one intersection)
   - **Δ > 0**: Line intersects circle at two points

4. **Check if intersections are on segment**:
   ```
   t₁ = (-B + √Δ) / (2A)
   t₂ = (-B - √Δ) / (2A)
   ```

   Collision occurs if either **t₁** or **t₂** is in **[0, 1]**

### Implementation

```cpp
// From include/spanny/rrt.hpp:163-190
bool in_collision(like::some_point auto const& p1,
                  like::some_point auto const& p2,
                  std::span<circle_t const> obstacles) {
  auto const A = distance_squared(p1, p2);

  return std::ranges::any_of(obstacles, [&](auto const& obstacle) {
    // Check endpoints first
    if (distance_between(p1, obstacle) <= obstacle.radius) {
      return true;
    }
    if (distance_between(p2, obstacle) <= obstacle.radius) {
      return true;
    }

    // Solve quadratic equation for line-circle intersection
    auto const B = 2 * ((p2.x - p1.x) * (p1.x - obstacle.x) +
                        (p2.y - p1.y) * (p1.y - obstacle.y));
    auto const C = distance_squared(p1, obstacle) -
                   obstacle.radius * obstacle.radius;

    // Check discriminant
    auto const discriminant = B * B - 4 * A * C;
    if (discriminant >= 0.) {
      auto const sqrt_d = std::sqrt(discriminant);
      auto const t1 = (-B + sqrt_d) / (2 * A);
      auto const t2 = (-B - sqrt_d) / (2 * A);

      // Check if either intersection point is on the segment [0, 1]
      if (is_between(t1, 0., 1.) or is_between(t2, 0., 1.)) {
        return true;
      }
    }

    return false;
  });
}
```

### Optimization Notes

1. **Early endpoint checks**: Avoids expensive quadratic solver if endpoints collide
2. **Squared distances**: Uses `distance_squared()` to avoid `sqrt()` where possible
3. **Short-circuit evaluation**: Returns immediately on first collision
4. **Precomputed A**: Factor A is constant for all obstacles

**Complexity**: O(m) where m = number of obstacles

## Performance Characteristics

### Scaling Behavior

From benchmark results (see [Performance Analysis](./performance-analysis.md)):

| Obstacles | Time per Check | Scaling |
|-----------|---------------|---------|
| 1         | ~27 ns        | Baseline |
| 10        | ~27 ns        | Constant |
| 100       | ~27 ns        | Constant |

The O(m) complexity is hidden by:
- Small obstacle counts in practice (< 100)
- Early termination on collision
- Modern CPU branch prediction

### Real-World Performance

In RRT planning with 100 nodes and 4 obstacles:
- **Simple scenario (no obstacles)**: ~0.18 ms total
- **Complex scenario (4 obstacles)**: ~0.22 ms total

Collision checking adds minimal overhead (~22% increase) for typical scenes.

## Numerical Considerations

### Floating-Point Precision

The implementation uses `double` precision throughout:
- Sufficient for meter-scale robotics
- Discriminant calculation is numerically stable
- Endpoint checks use `<=` to catch tangent cases

### Edge Cases

The algorithm correctly handles:
- **Zero-length segments**: A = 0 case works correctly
- **Tangent lines**: discriminant = 0 cases
- **Segment endpoint on circle**: Caught by endpoint checks
- **Multiple obstacles**: Any collision triggers failure

## Alternative Approaches

### Polygon Obstacles

For non-circular obstacles, consider:
- **GJK Algorithm**: General convex collision detection
- **SAT (Separating Axis Theorem)**: Fast for polygons
- **Minkowski Difference**: For complex shapes

### Spatial Acceleration

For many obstacles (> 1000):
- **Spatial hashing**: O(1) average case lookup
- **BVH (Bounding Volume Hierarchy)**: O(log m) queries
- **Quadtree/Octree**: Spatial partitioning

Current linear search is adequate for typical robotics scenes with < 100 obstacles.

## Testing

The collision detection is thoroughly tested in `test/test_rrt.cpp`:

```cpp
TEST(CollisionTests, DetectsLineCircleIntersection) {
  auto const p1 = spanny::position_t{-1, 0};
  auto const p2 = spanny::position_t{1, 0};
  auto const obstacles = std::array{
      spanny::circle_t{0, 0, 0.5}
  };

  EXPECT_TRUE(spanny::in_collision(p1, p2, obstacles));
}

TEST(CollisionTests, AllowsLineCircleMiss) {
  auto const p1 = spanny::position_t{-1, 1};
  auto const p2 = spanny::position_t{1, 1};
  auto const obstacles = std::array{
      spanny::circle_t{0, 0, 0.5}
  };

  EXPECT_FALSE(spanny::in_collision(p1, p2, obstacles));
}
```

## See Also

- [RRT Overview](./rrt-overview.md) - How collision checking fits into path planning
- [Performance Analysis](./performance-analysis.md) - Collision detection benchmarks
- [Custom Obstacles Example](../examples/custom-obstacles.md) - Creating complex scenes
- [Doxygen Reference](/doxygen/html/) - Complete API documentation

## References

- Ericson, C. (2004). *Real-Time Collision Detection*. CRC Press. Chapter 5: Basic Primitive Tests.
- Schneider, P. J., & Eberly, D. H. (2002). *Geometric Tools for Computer Graphics*. Morgan Kaufmann. Chapter 10: Intersection Methods.
