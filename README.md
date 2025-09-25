[![ci](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml) [![coverage](https://img.shields.io/badge/coverage-report-blue.svg)](https://github.com/griswaldbrooks/spanny3/actions)
# spanny3
Robot arm project for CppCon 2024 presentation.

# Development Environment

## Docker-Native Development Workflow

This project uses a **Docker-native approach** for development with the following principles:
- **No wrapper scripts**: Pure Docker Compose commands only
- **Manual UID/GID export**: Required due to Docker limitations, but clearly documented
- **Proper ownership**: Container user matches host user for seamless file access
- **Persistent caches**: Build artifacts and pre-commit caches survive container restarts
- **Enhanced security**: Minimal privileges instead of `privileged: true`

## Quick Start

### 1. Build the development image
```shell
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml build
```

### 2. Start interactive development container
```shell
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development
```

### 3. Build the project (inside container)
```shell
cmake -S src/spanny3/ -B artifacts/build
cmake --build artifacts/build
```

### 4. Run the application
```shell
./artifacts/build/rrt_cli
```

### 5. Run tests
```shell
ctest --test-dir artifacts/build --output-on-failure
```

## Development Commands

All commands should be run from inside the development container:

### Build Project
```shell
cmake -S src/spanny3 -B artifacts/build 
cmake --build artifacts/build
```

### Run Tests
```shell
ctest --test-dir artifacts/build --output-on-failure
```

### Run Linting
```shell
cd src/spanny3 && pre-commit run --all-files
```

## Container Features

- **Persistent volumes**: Build artifacts and caches survive container restarts
- **Proper ownership**: All files owned by your user (no root permission issues)
- **Isolated networking**: Custom bridge network instead of host mode
- **Security**: Minimal capabilities (no privileged mode)
- **Development tools**: clang-18, clang-format, pre-commit, neovim, git

## Why Manual UID/GID Export?

Docker has no native way to automatically detect host user IDs. The manual export approach is the cleanest **Docker-native** solution that:
- Works consistently across all systems
- Doesn't require wrapper scripts or complex automation
- Maintains clear visibility of what's happening
- Follows Docker Compose best practices

# coverage
Generate and view code coverage reports (run inside development container):

## Quick Coverage Summary
Build with coverage and view CLI summary:
```shell
cmake -S src/spanny3/ -B artifacts/build -DCMAKE_BUILD_TYPE=Coverage
cmake --build artifacts/build
cmake --build artifacts/build --target coverage
cat artifacts/build/coverage/reports/coverage.txt
```

## View Coverage in Browser
Coverage HTML reports are accessible from host since artifacts directory is mounted:
- **From host**: Open `~/.spanny3/build/coverage/reports/html/index.html` in your browser

## Individual Coverage Commands
- **Clean coverage data**: `cmake --build artifacts/build --target coverage-clean`
- **Run tests with coverage**: `cmake --build artifacts/build --target coverage-run`
- **Generate reports only**: `cmake --build artifacts/build --target coverage-report`
- **Full pipeline**: `cmake --build artifacts/build --target coverage`

## Adding New Libraries for Coverage
When adding new project libraries, include them in coverage by adding this block:
```cmake
add_library(my_new_lib SHARED
  src/my_new_lib.cpp
)

# Add to project libraries list for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  set(PROJECT_LIBRARIES "${PROJECT_LIBRARIES};my_new_lib" CACHE INTERNAL "List of project libraries for coverage")
endif()
```
The library will automatically be included in coverage reports with no changes to coverage commands.

# remove orphaned containers
```shell
docker compose -f compose.dev.yml down --remove-orphans
```

# start lsp
```shell
export PORT=5051
socat TCP-LISTEN:${PORT},fork,reuseaddr EXEC:"clangd -log=verbose --background-index --path-mappings='/host/path/to/source=/container/path/to/source'"

```
