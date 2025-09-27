# First Run

Run your first RRT path planning scenario with Spanny3.

## Quick Example

After [installation](installation.md), run the default scenario:

```bash
pixi run run-scenario
```

This executes the RRT planner with the configuration from `config/scenario.json`.

## Understanding the Output

The CLI outputs the planned path as a series of positions:

```
0.000, 0.000
0.450, 0.320
0.890, 0.640
...
4.780, 4.890
5.000, 5.000
```

Each line represents a node in the RRT tree, showing the `x, y` coordinates.

## Default Scenario

The default scenario (`config/scenario.json`) defines:

```json
{
  "start": [0.0, 0.0],
  "goal": [8.0, 8.0],
  "x_limits": [-10.0, 10.0],
  "y_limits": [-10.0, 10.0],
  "expansion_limit": 1000,
  "sample_distance": 0.5,
  "goal_probability": 0.05,
  "obstacles": [
    [2.0, 2.0, 1.0],
    [5.0, 3.0, 1.5],
    [3.0, 6.0, 1.0],
    [6.0, 6.0, 1.2]
  ]
}
```

### Parameters

- **start/goal**: Starting and goal positions `[x, y]`
- **x_limits/y_limits**: Planning space boundaries
- **expansion_limit**: Maximum number of nodes to explore
- **sample_distance**: Fixed distance for tree expansion
- **goal_probability**: Chance of sampling goal (0.05 = 5%)
- **obstacles**: Circular obstacles `[x, y, radius]`

## Creating Custom Scenarios

Create a new JSON file with your own configuration:

```json
{
  "start": [0.0, 0.0],
  "goal": [10.0, 10.0],
  "x_limits": [-15.0, 15.0],
  "y_limits": [-15.0, 15.0],
  "expansion_limit": 2000,
  "sample_distance": 0.8,
  "goal_probability": 0.1,
  "obstacles": [
    [5.0, 5.0, 2.0]
  ]
}
```

Run with your custom scenario:

```bash
pixi run build
./build/rrt_cli --scenario path/to/your_scenario.json
```

## Next Steps

- [RRT Overview](../algorithm/rrt-overview.md) - Understand the algorithm
- [Basic Planning Example](../examples/basic-planning.md) - Programming examples
- [Custom Obstacles](../examples/custom-obstacles.md) - Creating complex environments

## Troubleshooting

### Planning Failed

If you see `RRT failed to reach goal`, try:
- Increase `expansion_limit`
- Decrease obstacle sizes or remove some
- Increase `goal_probability`
- Ensure goal is not inside an obstacle

### No Output

If the program exits without output:
- Check that your scenario file is valid JSON
- Verify the file path is correct
- Run with `pixi run build` first to ensure binary is up-to-date
