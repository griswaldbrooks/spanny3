# Claude Code Configuration

## Project Overview

Spanny3 is a **C++ robotics project** implementing a **Rapidly-Exploring Random Tree (RRT)** path planning algorithm. Created for a CppCon 2024 presentation, it demonstrates modern C++23 features in a practical robotics context.

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

## Build Commands
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build && cmake --build build"
```

## Test Commands
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build && cmake --build build && ctest --test-dir build --output-on-failure"
```

## Lint Commands
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cd src/spanny3 && pre-commit run --all-files"
```

## Combined Build, Test, and Lint
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build && cmake --build build && ctest --test-dir build --output-on-failure && cd src/spanny3 && pre-commit run --all-files"
```

## Hooks Configuration

### Build Hook
Automatically run build after code changes:
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build && cmake --build build"
```

### Test Hook
Run tests after successful builds:
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build && cmake --build build && ctest --test-dir build --output-on-failure"
```

### Lint Hook
Run linting/formatting checks:
```bash
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cd src/spanny3 && pre-commit run --all-files"
```

## Usage Notes
- **Language**: C++23 with Clang 18 compiler and libc++ standard library
- **Build System**: CMake with modern target management
- **Testing**: CTest with GoogleTest/GoogleMock framework
- **Code Quality**: Pre-commit hooks with clang-format, codespell, and other tools
- **Development Environment**: Docker containerized development via `compose.dev.yml`
- **Container Structure**: Project mounted at `/home/griswald/ws/src/spanny3/` in container
- **Key Files**:
  - `config/scenario.json`: Default planning scenario with obstacles
  - `src/spanny3/`: CMake project root in container
  - `build/`: Build output directory (created during build process)

## Architecture Notes for Agents
- **Error Handling**: Uses `std::expected<T, std::string>` throughout for recoverable errors
- **Concepts**: `some_point` and `some_random_generator` enable generic programming
- **Testing Strategy**: Mock random generators for deterministic testing of stochastic algorithms
- **Collision Detection**: Line-circle intersection using quadratic equation solving
- **Path Planning**: RRT with configurable sampling distance, goal probability, and expansion limits