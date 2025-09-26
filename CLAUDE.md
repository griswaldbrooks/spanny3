# Claude Code Configuration

## Project Overview

Spanny3 is a **C++ robotics project** implementing a **Rapidly-Exploring Random Tree (RRT)** path planning algorithm. Created for a CppCon 2024 presentation, it demonstrates modern C++23 features in a practical robotics context.

## Docker Development Philosophy

This project follows a **Docker-native development approach** with these principles:

### Core Philosophy
- **Pure Docker Compose**: No wrapper scripts, aliases, or Makefiles
- **Manual UID/GID Export**: Explicit and transparent (Docker has no native alternative)
- **Proper File Ownership**: Container user matches host user for seamless development
- **Enhanced Security**: Minimal privileges instead of broad `privileged: true`
- **Persistent Caches**: Build artifacts survive container restarts

### Compose File Structure
- **Single compose file**: `compose.dev.yml` for development
- **Named project**: `name: spanny3-dev` prevents container conflicts
- **External networking**: Custom bridge network instead of host mode
- **Volume management**: Named volumes for artifacts and caches
- **Security hardening**: Specific capabilities only when needed

### Key Components
- **Path Planning Engine** (`src/rrt.cpp`, `include/spanny/rrt.hpp`): Core RRT algorithm with C++23 features
- **CLI Tool** (`src/rrt_cli.cpp`): Command-line interface for running path planning scenarios
- **Type System** (`include/spanny/like.hpp`): C++20 concepts for behavioral constraints
- **Tests** (`test/test_rrt.cpp`): GoogleTest framework with mocking for stochastic components

### Technical Highlights
- **Modern C++23**: Uses `std::expected`, concepts, `std::ranges`, and designated initializers
- **Clean Architecture**: Separates geometric primitives, planning context, and sampling strategies
- **Robust Testing**: Dependency injection enables deterministic testing of stochastic algorithms
- **Docker Development**: Containerized build environment with Clang 18 and libc++

### Dependencies
- **nlohmann::json**: Configuration file parsing
- **cxxopts**: Command-line argument parsing
- **GoogleTest/GoogleMock**: Testing framework
- **Pre-commit**: Code formatting and linting

## Development Workflow Commands

### Option 1: Pixi Workflow (Recommended for Local Development)

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

### Option 2: Docker Workflow (CI/CD Compatible)

#### Container Setup
Start the development environment:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development
```

### Build Commands (inside container)
```bash
cmake -S . -B build && cmake --build build
```

### Test Commands (inside container)
```bash
ctest --test-dir build --output-on-failure
```

### Lint Commands (inside container)
```bash
pre-commit run --all-files
```

### One-Shot Commands (from host)
For automation or CI-like workflows:

**Build:**
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build && cmake --build build"
```

**Test:**
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "ctest --test-dir build --output-on-failure"
```

**Lint:**
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "pre-commit run --all-files"
```

**Combined Pipeline:**
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure && pre-commit run --all-files"
```

## Coverage Commands

### Coverage Build (inside container)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build
```

### Coverage Analysis (inside container)
Run full coverage analysis (build, test, and generate reports):
```bash
cmake --build build --target coverage
```

### Coverage Reports Only (inside container)
Generate coverage reports from existing profile data:
```bash
cmake --build build --target coverage-report
```

### Coverage Clean (inside container)
Clean coverage data and start fresh:
```bash
cmake --build build --target coverage-clean
```

### One-Shot Coverage (from host)
Full coverage pipeline:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build && cmake --build build --target coverage"
```

### View Coverage Reports
Access coverage reports after running coverage analysis:
- **HTML Report**: Open `build/coverage/reports/html/index.html` in browser
- **Text Summary**: View `build/coverage/reports/coverage.txt`

## Hooks Configuration

### Pixi-based Hooks (Recommended)

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

### Docker-based Hooks (Alternative)

#### Build Hook
Automatically run build after code changes:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build && cmake --build build"
```

#### Test Hook
Run tests after successful builds:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure"
```

#### Lint Hook
Run linting/formatting checks:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "pre-commit run --all-files"
```

#### Coverage Hook
Run full coverage analysis:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S . -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build && cmake --build build --target coverage"
```

## Usage Notes
- **Language**: C++23 with Clang 18 compiler and libc++ standard library
- **Build System**: CMake with modern target management and presets
- **Testing**: CTest with GoogleTest/GoogleMock framework
- **Code Quality**: Pre-commit hooks with clang-format, codespell, and other tools
- **Development Environments**:
  - **Pixi**: Cross-platform package manager for local development (Linux and macOS)
  - **Docker**: Containerized development via `compose.dev.yml` for CI/CD consistency
- **Pixi Structure**:
  - Project root: Current directory
  - Build artifacts: `build/` directory
  - Environment: `.pixi/` (auto-managed, git-ignored)
  - Platform-specific: Linux requires libcxx/libcxxabi/compiler-rt; macOS uses system libc++
- **Docker Container Structure**:
  - Project root: `/home/${USER}/ws/src/spanny3/`
  - Build artifacts: `/home/${USER}/ws/artifacts/`
  - All files owned by container user (matches host user)
- **Key Files**:
  - `pixi.toml`: Pixi configuration with dependencies and tasks
  - `CMakePresets.json`: CMake presets for different build configurations
  - `config/scenario.json`: Default planning scenario with obstacles
  - `compose.dev.yml`: Docker Compose configuration for development
  - `build/` or `artifacts/build/`: Build output directory

## Architecture Notes for Agents
- **Error Handling**: Uses `std::expected<T, std::string>` throughout for recoverable errors
- **Concepts**: `some_point` and `some_random_generator` enable generic programming
- **Testing Strategy**: Mock random generators for deterministic testing of stochastic algorithms
- **Collision Detection**: Line-circle intersection using quadratic equation solving
- **Path Planning**: RRT with configurable sampling distance, goal probability, and expansion limits

## Adding New Libraries
When adding new project libraries, include them in coverage by adding this block after the library definition:
```cmake
# Add to project libraries list for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  set(PROJECT_LIBRARIES "${PROJECT_LIBRARIES};library_name" CACHE INTERNAL "List of project libraries for coverage")
endif()
```
The library will automatically be included in coverage reports. Coverage is filtered to show only project code (excludes third_party, test, build artifacts).
