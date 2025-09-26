# Claude Code Configuration

## Project Overview

Spanny3 is a **C++ robotics project** implementing a **Rapidly-Exploring Random Tree (RRT)** path planning algorithm. Created for a CppCon 2024 presentation, it demonstrates modern C++23 features in a practical robotics context.

## Pixi Development Philosophy

This project uses **Pixi** for package management and development workflows with these principles:

### Core Philosophy
- **Simple Commands**: `pixi run dev` for complete development cycle
- **Cross-Platform**: Works on Linux and macOS natively
- **Reproducible**: Lock file ensures consistent environments
- **Fast Iteration**: No container overhead for local development
- **CI/CD Integration**: Same commands work locally and in GitHub Actions

### Key Components
- **Path Planning Engine** (`src/rrt.cpp`, `include/spanny/rrt.hpp`): Core RRT algorithm with C++23 features
- **CLI Tool** (`src/rrt_cli.cpp`): Command-line interface for running path planning scenarios
- **Type System** (`include/spanny/like.hpp`): C++20 concepts for behavioral constraints
- **Tests** (`test/test_rrt.cpp`): GoogleTest framework with mocking for stochastic components

### Technical Highlights
- **Modern C++23**: Uses `std::expected`, concepts, `std::ranges`, and designated initializers
- **Clean Architecture**: Separates geometric primitives, planning context, and sampling strategies
- **Robust Testing**: Dependency injection enables deterministic testing of stochastic algorithms
- **Pixi Development**: Native development environment with Clang 18 and libc++

### Dependencies
- **nlohmann::json**: Configuration file parsing
- **cxxopts**: Command-line argument parsing
- **GoogleTest/GoogleMock**: Testing framework
- **Pre-commit**: Code formatting and linting

## Development Workflow Commands

### Pixi Workflow

#### Initial Setup
Install Pixi and set up the environment:
```bash
# Install Pixi (one-time setup)
curl -fsSL https://pixi.sh/install.sh | bash

# Install dependencies and activate environment
pixi install
pixi shell  # Or use 'pixi run <command>' for individual commands
```

#### Common Pixi Commands
```bash
# Quick development cycle
pixi run dev              # Configure, build, and test in one command

# Individual tasks
pixi run build           # Build the project
pixi run test            # Run tests
pixi run coverage        # Generate coverage report
pixi run lint            # Run all linters and formatters

# Release builds
pixi run build-release   # Build optimized version
pixi run configure-release && pixi run test  # Test release build

# Utility commands
pixi run run-scenario    # Run with default scenario
pixi task list           # Show all available tasks
pixi info                # Show environment information
```

#### CMake Preset Usage with Pixi
```bash
# Using CMake presets directly
cmake --preset pixi-debug        # Configure debug build
cmake --build --preset pixi-debug # Build debug version
ctest --preset pixi-test          # Run tests

# Coverage workflow with presets
cmake --preset pixi-coverage
cmake --build --preset pixi-coverage
ctest --preset pixi-test-coverage
```

## Coverage Commands

Run full coverage analysis:
```bash
pixi run coverage
```

View coverage reports:
- **HTML Report**: Open `build/coverage/reports/html/index.html` in browser
- **Text Summary**: View `build/coverage/reports/coverage.txt`

## Hooks Configuration

### Pixi-based Hooks

#### Build Hook
Automatically run build after code changes:
```bash
pixi run build
```

#### Test Hook
Run tests after successful builds:
```bash
pixi run test
```

#### Lint Hook
Run linting/formatting checks:
```bash
pixi run lint
```

#### Coverage Hook
Run full coverage analysis:
```bash
pixi run coverage
```

#### Full Development Cycle Hook
Configure, build, and test in sequence:
```bash
pixi run dev
```

## Usage Notes
- **Language**: C++23 with Clang 18 compiler and libc++ standard library
- **Build System**: CMake with modern target management and presets
- **Testing**: CTest with GoogleTest/GoogleMock framework
- **Code Quality**: Pre-commit hooks with clang-format, codespell, and other tools
- **Development Environment**:
  - **Pixi**: Cross-platform package manager for local development and CI/CD (Linux and macOS)
  - **No Docker**: Docker workflow was removed in favor of Pixi-only approach
- **Pixi Structure**:
  - Project root: Current directory
  - Build artifacts: `build/` directory
  - Environment: `.pixi/` (auto-managed, git-ignored)
  - Lock file: `pixi.lock` (committed to repo for reproducible CI builds)
  - Platform-specific: Linux requires libcxx/libcxxabi/compiler-rt; macOS uses system libc++
- **Key Files**:
  - `pixi.toml`: Pixi configuration with dependencies and tasks
  - `pixi.lock`: Lock file for reproducible dependency versions (must be committed)
  - `CMakePresets.json`: CMake presets for different build configurations
  - `config/scenario.json`: Default planning scenario with obstacles
  - `build/`: Build output directory
  - `.github/workflows/ci.yml`: CI/CD using Pixi on Linux and macOS

## Architecture Notes for Agents
- **Error Handling**: Uses `std::expected<T, std::string>` throughout for recoverable errors
- **Concepts**: `some_point` and `some_random_generator` enable generic programming
- **Testing Strategy**: Mock random generators for deterministic testing of stochastic algorithms
- **Collision Detection**: Line-circle intersection using quadratic equation solving
- **Path Planning**: RRT with configurable sampling distance, goal probability, and expansion limits

## Known Issues and Next Steps

See [IMPROVEMENT_PLAN.md](IMPROVEMENT_PLAN.md) for detailed roadmap. Priority items:

### Critical Bug (src/rrt.cpp:118)
Edge relationship is reversed - `tree.edges.emplace_back(sample.id, closest.id, cost)` should be `tree.edges.emplace_back(closest.id, sample.id, cost)` for correct parent-child relationship.

### Missing Functionality
- Path extraction functionality not implemented (tree is generated but path is not extracted)
- CLI uses hardcoded container path instead of relative path

### CI/CD Notes
- CI runs on Linux and macOS (ARM64 & x64) using Pixi
- Three jobs: build (6 configs), coverage (Linux only), lint
- All commands: `pixi run dev`, `pixi run coverage`, `pixi run lint`
- If Pixi version changes locally, may need to update CI to match
- Always commit `pixi.lock` after dependency changes

## Adding New Libraries
When adding new project libraries, include them in coverage by adding this block after the library definition:
```cmake
# Add to project libraries list for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  set(PROJECT_LIBRARIES "${PROJECT_LIBRARIES};library_name" CACHE INTERNAL "List of project libraries for coverage")
endif()
```
The library will automatically be included in coverage reports. Coverage is filtered to show only project code (excludes third_party, test, build artifacts).
