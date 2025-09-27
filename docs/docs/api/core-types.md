# Core Types

Fundamental data structures for 2D path planning in Spanny3.

## position_t

Represents a 2D position in the planning space.

```cpp
struct position_t {
  double x;  // X coordinate
  double y;  // Y coordinate
};
```

### Example Usage

```cpp
#include "spanny/rrt.hpp"

auto start = spanny::position_t{0.0, 0.0};
auto goal = spanny::position_t{5.0, 5.0};

// Calculate distance
auto dist = spanny::distance_between(start, goal);  // ~7.07
```

---

## displacement_t

Represents a displacement (vector) in 2D space.

```cpp
struct displacement_t {
  double x;  // X component
  double y;  // Y component
};
```

### Operators

The displacement type supports standard vector operations:

```cpp
auto d1 = spanny::displacement_t{1.0, 2.0};
auto d2 = spanny::displacement_t{3.0, 4.0};

// Addition
auto sum = d1 + d2;  // {4.0, 6.0}

// Subtraction
auto diff = d2 - d1;  // {2.0, 2.0}

// Scalar multiplication
auto scaled = d1 * 2.0;  // {2.0, 4.0}

// Scalar division
auto divided = d1 / 2.0;  // {0.5, 1.0}
```

### Utility Functions

```cpp
// Calculate magnitude
auto mag = spanny::magnitude(d1);  // sqrt(1² + 2²) = ~2.24

// Normalize to unit vector
auto unit = spanny::normalize(d1);  // {0.447, 0.894}
```

### Position-Displacement Operations

```cpp
auto pos = spanny::position_t{1.0, 1.0};
auto disp = spanny::displacement_t{2.0, 3.0};

// Add displacement to position
auto new_pos = pos + disp;  // {3.0, 4.0}

// Subtract displacement from position
auto moved = pos - disp;  // {-1.0, -2.0}

// Difference between positions yields displacement
auto p1 = spanny::position_t{5.0, 5.0};
auto p2 = spanny::position_t{2.0, 1.0};
auto diff = p1 - p2;  // displacement_t{3.0, 4.0}
```

---

## node_t

Represents a node in the RRT tree.

```cpp
struct node_t {
  explicit node_t(position_t p);

  node_id_t id;          // Unique identifier (hash of position)
  position_t position;   // 2D position
};
```

### Example Usage

```cpp
auto pos = spanny::position_t{3.0, 4.0};
auto node = spanny::node_t{pos};

// Node ID is automatically generated from position hash
std::cout << "Node ID: " << node.id << "\n";
std::cout << "Position: (" << node.position.x << ", "
          << node.position.y << ")\n";
```

### Node Identifier

The `node_id_t` is a `std::size_t` computed by hashing the position:

```cpp
using node_id_t = std::size_t;

// Hash is used for efficient node lookup and uniqueness
auto id = std::hash<spanny::position_t>{}(pos);
```

---

## edge_t

Represents an edge connecting two nodes in the tree.

```cpp
struct edge_t {
  node_id_t parent;  // Parent node ID
  node_id_t child;   // Child node ID
  double cost;       // Edge cost (distance)
};
```

### Example Usage

```cpp
auto parent_node = spanny::node_t{spanny::position_t{0.0, 0.0}};
auto child_node = spanny::node_t{spanny::position_t{1.0, 1.0}};

auto cost = spanny::distance_between(
    parent_node.position,
    child_node.position
);  // ~1.41

auto edge = spanny::edge_t{
    .parent = parent_node.id,
    .child = child_node.id,
    .cost = cost
};
```

---

## tree_t

Container for the RRT tree structure.

```cpp
struct tree_t {
  std::vector<node_t> nodes;  // All nodes in the tree
  std::vector<edge_t> edges;  // All edges connecting nodes
};
```

### Example Usage

```cpp
// Tree starts with root node
auto tree = spanny::tree_t{};
tree.nodes.emplace_back(spanny::position_t{0.0, 0.0});

// Add child node
tree.nodes.emplace_back(spanny::position_t{0.5, 0.5});

// Connect with edge
auto cost = spanny::distance_between(
    tree.nodes[0].position,
    tree.nodes[1].position
);
tree.edges.emplace_back(
    tree.nodes[0].id,  // parent
    tree.nodes[1].id,  // child
    cost
);

std::cout << "Tree size: " << tree.nodes.size() << " nodes\n";
std::cout << "Edge count: " << tree.edges.size() << " edges\n";
```

---

## circle_t

Represents a circular obstacle in the planning space.

```cpp
struct circle_t {
  double x;       // X coordinate of center
  double y;       // Y coordinate of center
  double radius;  // Radius of the circle
};
```

### Example Usage

```cpp
// Single obstacle
auto obstacle = spanny::circle_t{
    .x = 5.0,
    .y = 5.0,
    .radius = 2.0
};

// Multiple obstacles
std::vector<spanny::circle_t> obstacles{
    {2.0, 2.0, 1.0},   // Small obstacle at (2,2)
    {5.0, 3.0, 1.5},   // Medium obstacle at (5,3)
    {3.0, 6.0, 1.0},   // Small obstacle at (3,6)
};

// Check collision
auto p1 = spanny::position_t{0.0, 0.0};
auto p2 = spanny::position_t{10.0, 10.0};
bool collides = spanny::in_collision(p1, p2, obstacles);
```

---

## bounds_t

Defines a numerical range with minimum and maximum values.

```cpp
struct bounds_t {
  double min;  // Minimum value
  double max;  // Maximum value
};
```

### Example Usage

```cpp
// Define planning space bounds
auto x_limits = spanny::bounds_t{-10.0, 10.0};  // -10 to +10
auto y_limits = spanny::bounds_t{-10.0, 10.0};

// Bounds are used in planning_context_t
spanny::planning_context_t context{
    .x_limits = x_limits,
    .y_limits = y_limits,
    // ...
};
```

---

## See Also

- [Planning Context](planning-context.md) - Configuration for RRT planning
- [RRT Planner](rrt-planner.md) - Using the RRT algorithm
- [Doxygen Reference](/doxygen/) - Complete API documentation
