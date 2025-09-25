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
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build && cmake --build artifacts/build"
```

## Test Commands
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build && cmake --build artifacts/build && ctest --test-dir artifacts/build --output-on-failure"
```

## Lint Commands
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cd src/spanny3 && pre-commit run --all-files"
```

## Combined Build, Test, and Lint
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build && cmake --build artifacts/build && ctest --test-dir artifacts/build --output-on-failure && cd src/spanny3 && pre-commit run --all-files"
```

## Coverage Commands

### Coverage Build
Build with coverage instrumentation:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build -DCMAKE_BUILD_TYPE=Coverage && cmake --build artifacts/build"
```

### Coverage Analysis
Run full coverage analysis (build, test, and generate reports):
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build -DCMAKE_BUILD_TYPE=Coverage && cmake --build artifacts/build && cmake --build artifacts/build --target coverage"
```

### Coverage Reports Only
Generate coverage reports from existing profile data:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake --build artifacts/build --target coverage-report"
```

### Coverage Clean
Clean coverage data and start fresh:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake --build artifacts/build --target coverage-clean"
```

### View Coverage Reports
Access HTML coverage reports (after running coverage analysis):
- **HTML Report**: Open `~/.spanny3/build/coverage/reports/html/index.html` in browser
- **Text Summary**: View `~/.spanny3/build/coverage/reports/coverage.txt`

## Hooks Configuration

### Build Hook
Automatically run build after code changes:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build && cmake --build artifacts/build"
```

### Test Hook
Run tests after successful builds:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build && cmake --build artifacts/build && ctest --test-dir artifacts/build --output-on-failure"
```

### Lint Hook
Run linting/formatting checks:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cd src/spanny3 && pre-commit run --all-files"
```

### Coverage Hook
Run full coverage analysis:
```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B artifacts/build -DCMAKE_BUILD_TYPE=Coverage && cmake --build artifacts/build && cmake --build artifacts/build --target coverage"
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

## Adding New Libraries
When adding new project libraries, include them in coverage by adding this block after the library definition:
```cmake
# Add to project libraries list for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  set(PROJECT_LIBRARIES "${PROJECT_LIBRARIES};library_name" CACHE INTERNAL "List of project libraries for coverage")
endif()
```
The library will automatically be included in coverage reports. Coverage is filtered to show only project code (excludes third_party, test, build artifacts).
