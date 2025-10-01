---
sidebar_position: 3
---

# Contributing Guide

Guidelines for contributing to Spanny3, including code style, testing requirements, and the pull request process.

## Getting Started

### Fork and Clone

1. Fork the repository on GitHub
2. Clone your fork:
   ```bash
   git clone https://github.com/YOUR_USERNAME/spanny3.git
   cd spanny3
   ```

3. Set up the development environment:
   ```bash
   pixi install
   pixi run dev
   ```

### Create a Branch

Create a feature branch for your changes:

```bash
git checkout -b feature/my-awesome-feature
```

Use descriptive branch names:
- `feature/add-rrt-star` - New features
- `fix/collision-bug` - Bug fixes
- `docs/api-examples` - Documentation updates
- `refactor/tree-structure` - Code refactoring

## Code Style

### C++23 Standards

Spanny3 uses modern C++23 features:

- **Designated initializers**: For struct construction
  ```cpp
  auto context = spanny::planning_context_t{
      .x_limits = {-10., 10.},
      .y_limits = {-10., 10.},
      .expansion_limit = 1000,
      .sample_distance = 1.,
      .goal_probability = 0.1,
      .obstacles = {}
  };
  ```

- **std::expected**: For error handling (no exceptions)
  ```cpp
  std::expected<tree_t, std::string> plan(/*...*/) {
    if (invalid_input) {
      return std::unexpected(std::string{"Invalid input"});
    }
    return tree;
  }
  ```

- **Concepts**: For generic programming
  ```cpp
  template <like::some_random_generator random_generator_t>
  struct rrt_t { /*...*/ };
  ```

- **Ranges**: For expressive algorithms
  ```cpp
  std::ranges::sort(tree.nodes, {}, &node_t::id);
  std::ranges::transform(input, std::back_inserter(output), transform_fn);
  ```

### Formatting with clang-format

Code is automatically formatted with clang-format 18.

**Format your changes:**
```bash
pixi run format
```

**Check formatting:**
```bash
pixi run lint
```

### Naming Conventions

Follow these naming patterns:

**Types**: `snake_case` with `_t` suffix
```cpp
struct position_t { double x, y; };
struct planning_context_t { /*...*/ };
using node_id_t = std::size_t;
```

**Functions**: `snake_case`
```cpp
double distance_between(position_t const& p1, position_t const& p2);
displacement_t normalize(displacement_t const& d);
```

**Variables**: `snake_case`
```cpp
auto start_position = position_t{0., 0.};
auto expansion_limit = 1000;
```

**Private members**: `snake_case` with trailing underscore
```cpp
class planner {
 private:
  random_generator_t& random_generator_;
  std::size_t iteration_count_;
};
```

**Constants**: `snake_case`
```cpp
constexpr auto default_sample_distance = 0.5;
constexpr auto max_iterations = 10000;
```

### Documentation Comments

Use Doxygen-style comments for public APIs:

```cpp
/**
 * @brief Project a point from a start in a direction for a distance.
 *
 * @param origin is the starting point for the projection
 * @param target to project towards
 * @param distance from origin to project
 * @returns a new point, @p distance away from @p origin in the direction of @p target
 */
auto project_towards(like::some_point auto const& origin,
                     like::some_point auto const& target,
                     double distance);
```

Document:
- **Brief description**: What the function does
- **Parameters**: Purpose of each parameter
- **Return value**: What is returned
- **Exceptions**: What errors can occur (use `@note` for `std::expected`)

## Testing Requirements

### Write Tests for New Code

Every new feature or bug fix requires tests:

```cpp
TEST(MyFeature, BasicBehavior) {
  // GIVEN - Setup
  auto input = create_test_input();

  // WHEN - Execute
  auto result = my_function(input);

  // THEN - Verify
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), expected_output);
}
```

### Test Coverage

Maintain high test coverage:

```bash
pixi run coverage
```

Open `build/coverage/reports/html/index.html` to verify your changes are covered.

**Target**: 80%+ line coverage for new code

### Test Edge Cases

Test boundary conditions and error cases:

```cpp
TEST(Planning, EmptyBounds) {
  auto context = spanny::planning_context_t{
      .x_limits = {0., 0.},  // Zero-width bounds
      .y_limits = {0., 0.},
      // ...
  };
  auto result = planner(start, goal, context);
  EXPECT_FALSE(result.has_value());
}

TEST(Planning, StartEqualsGoal) {
  auto pos = spanny::position_t{0., 0.};
  auto result = planner(pos, pos, context);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value().nodes.size(), 2);  // Start + goal
}
```

### Run Tests Before Submitting

```bash
pixi run dev  # Build and test
pixi run lint  # Check code style
```

All tests must pass before creating a pull request.

## Pre-commit Hooks

### Installing Hooks

Pre-commit hooks enforce code quality automatically:

```bash
pixi run -e dev pre-commit install
```

This installs hooks that run on every commit.

### Hook Checks

The pre-commit configuration includes:

- **clang-format**: C++ code formatting
- **cmake-format**: CMakeLists.txt formatting
- **codespell**: Spell checking in comments
- **check-yaml**: YAML file validation
- **trailing-whitespace**: Remove trailing spaces
- **end-of-file-fixer**: Ensure files end with newline

### Running Hooks Manually

```bash
# Run all hooks on all files
pixi run lint

# Run specific hook
pre-commit run clang-format --all-files

# Skip hooks for a commit (not recommended)
git commit --no-verify
```

## Pull Request Process

### Before Creating a PR

1. **Update from main:**
   ```bash
   git checkout main
   git pull upstream main
   git checkout your-feature-branch
   git rebase main
   ```

2. **Run full test suite:**
   ```bash
   pixi run dev
   pixi run lint
   pixi run coverage
   ```

3. **Update documentation:**
   - Add/update docstrings for new functions
   - Update relevant markdown docs if needed
   - Run `npm start` in `docs/` to preview changes

4. **Commit changes:**
   ```bash
   git add .
   git commit -m "Add feature: brief description

   Detailed explanation of changes and why they were made.
   Addresses issue #123."
   ```

### Creating the PR

1. Push your branch:
   ```bash
   git push origin feature/my-awesome-feature
   ```

2. Open a pull request on GitHub

3. Fill out the PR template:
   - **Description**: What does this PR do?
   - **Motivation**: Why is this change needed?
   - **Testing**: How was it tested?
   - **Screenshots**: If applicable (e.g., documentation changes)

### PR Title Format

Use clear, descriptive titles:

```
Add RRT* variant for optimal path planning
Fix edge relationship bug in tree expansion
Update API documentation with usage examples
Refactor collision detection for performance
```

### PR Description Template

```markdown
## Summary
Brief description of what this PR does.

## Motivation
Why is this change needed? What problem does it solve?

## Changes
- Change 1
- Change 2
- Change 3

## Testing
How was this tested?
- [ ] Unit tests added/updated
- [ ] Integration tests pass
- [ ] Coverage report reviewed

## Documentation
- [ ] Code comments updated
- [ ] API documentation updated
- [ ] User guide updated (if applicable)

## Checklist
- [ ] Tests pass (`pixi run test`)
- [ ] Linting passes (`pixi run lint`)
- [ ] Coverage maintained (`pixi run coverage`)
- [ ] Documentation updated
```

### Code Review Process

1. **Automated checks**: CI runs tests, linting, and coverage
2. **Manual review**: Maintainers review code quality and design
3. **Feedback**: Address review comments
4. **Approval**: At least one maintainer approval required
5. **Merge**: Squash and merge to main

### Addressing Review Feedback

```bash
# Make requested changes
git add .
git commit -m "Address review feedback: improve error messages"

# Push updates
git push origin feature/my-awesome-feature
```

The PR updates automatically.

## Building Documentation

### Local Preview

```bash
cd docs
npm install
npm start
```

Opens `http://localhost:3000` with live reload.

### Building for Production

```bash
cd docs
npm run build
```

Output goes to `docs/build/` directory.

### Adding Documentation

1. Create markdown file in `docs/docs/`
2. Add frontmatter:
   ```markdown
   ---
   sidebar_position: 1
   ---

   # Your Page Title
   ```

3. Update `docs/sidebars.ts` if adding new sections

4. Preview changes locally before committing

## Continuous Integration

### CI Workflow

GitHub Actions runs on every push and PR:

1. **Build**: Compiles on Linux and macOS (ARM64 & x64)
2. **Test**: Runs full test suite
3. **Lint**: Checks code style
4. **Coverage**: Generates coverage reports (Linux only)
5. **Benchmark**: Runs performance benchmarks (Linux only)
6. **Deploy Docs**: Builds and deploys documentation (main branch only)

### Required CI Checks

All of these must pass:

- Build succeeds on all platforms
- All tests pass
- Linting has no errors
- Coverage meets threshold (80%+)

### Viewing CI Results

1. Go to your PR on GitHub
2. Check "Checks" tab for detailed results
3. Click failed checks to see logs
4. Fix issues and push updates

## Issue Guidelines

### Reporting Bugs

Include:
- **Description**: What went wrong?
- **Steps to reproduce**: How can we reproduce it?
- **Expected behavior**: What should happen?
- **Actual behavior**: What actually happened?
- **Environment**: OS, Pixi version, compiler version
- **Logs**: Error messages and stack traces

### Requesting Features

Include:
- **Use case**: What problem does this solve?
- **Proposed solution**: How should it work?
- **Alternatives**: Other approaches considered?
- **Additional context**: Examples, references, related issues

### Asking Questions

- Check documentation first
- Search existing issues
- Provide context about what you're trying to do
- Include relevant code snippets

## Communication

### Getting Help

- **GitHub Discussions**: General questions and ideas
- **GitHub Issues**: Bug reports and feature requests
- **Pull Requests**: Code contributions and reviews

### Code of Conduct

Be respectful, inclusive, and constructive:
- Welcome newcomers and beginners
- Provide helpful feedback on PRs
- Be patient with questions
- Focus on the code, not the person
- Assume good intentions

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

## Recognition

Contributors are recognized in:
- GitHub contributors page
- Release notes for significant contributions
- Special thanks in documentation

Thank you for contributing to Spanny3!

## See Also

- [Testing Guide](testing-guide.md) - How to write and run tests
- [Pixi Workflow](pixi-workflow.md) - Advanced development workflows
- [Code Style Guide](https://google.github.io/styleguide/cppguide.html) - Google C++ Style Guide (adapted)
- [GitHub Flow](https://guides.github.com/introduction/flow/) - Branching workflow
