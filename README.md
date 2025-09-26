[![ci](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml) [![coverage](https://img.shields.io/badge/coverage-report-blue.svg)](https://github.com/griswaldbrooks/spanny3/actions)
# spanny3
Robot arm project for CppCon 2024 presentation.

# Development Environment

## Choose Your Development Workflow

This project supports two development workflows:

### 🚀 [Pixi Development](PIXI_DEVELOPMENT.md) (Recommended for Local Development)
- **Quick setup**: One command installs everything
- **Native performance**: No container overhead
- **Better IDE integration**: Debuggers and tools work naturally
- **Cross-platform**: Works on Linux and macOS (Windows not yet supported)

**Quick Start with Pixi:**
```bash
# Install Pixi (one-time)
curl -fsSL https://pixi.sh/install.sh | bash

# Setup and build
pixi install
pixi run dev
```

[**→ Full Pixi Development Guide**](PIXI_DEVELOPMENT.md)

### 🐳 Docker-Native Development Workflow (For CI/CD Compatibility)

This project uses a **Docker-native approach** for development with the following principles:
- **No wrapper scripts**: Pure Docker Compose commands only
- **Manual UID/GID export**: Required due to Docker limitations, but clearly documented
- **Proper ownership**: Container user matches host user for seamless file access
- **Persistent caches**: Build artifacts and pre-commit caches survive container restarts
- **Enhanced security**: Minimal privileges instead of `privileged: true`

## Which Workflow Should I Use?

| Use Case | Recommended Workflow | Why |
|----------|---------------------|-----|
| Daily development on your machine | Pixi | Faster, simpler commands, better IDE support |
| Quick prototyping and testing | Pixi | No container overhead |
| CI/CD pipelines | Docker | Already configured in GitHub Actions |
| Restricted environments | Docker | When you can't install Pixi |
| Production deployment | Docker | Container isolation |

## Quick Start with Docker

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
cmake -S . -B build
cmake --build build
```

### 4. Run the application
```shell
./build/rrt_cli
```

### 5. Run tests
```shell
ctest --test-dir build --output-on-failure
```

## Development Commands

All commands should be run from inside the development container:

### Build Project
```shell
cmake -S . -B build
cmake --build build
```

### Run Tests
```shell
ctest --test-dir build --output-on-failure
```

### Run Linting
```shell
pre-commit run --all-files
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

# Coverage
Generate and view code coverage reports (run inside development container):

## Quick Coverage Summary
Build with coverage and view CLI summary:
```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Coverage
cmake --build build
cmake --build build --target coverage
cat build/coverage/reports/coverage.txt
```

## View Coverage in Browser
Coverage HTML reports are accessible from host:
- **From host**: Open `build/coverage/reports/html/index.html` in your browser

## Individual Coverage Commands
- **Clean coverage data**: `cmake --build build --target coverage-clean`
- **Run tests with coverage**: `cmake --build build --target coverage-run`
- **Generate reports only**: `cmake --build build --target coverage-report`
- **Full pipeline**: `cmake --build build --target coverage`

# remove orphaned containers
```shell
docker compose -f compose.dev.yml down --remove-orphans
```

# start lsp
```shell
export PORT=5051
socat TCP-LISTEN:${PORT},fork,reuseaddr EXEC:"clangd -log=verbose --background-index --path-mappings='/host/path/to/source=/container/path/to/source'"

```

# Documentation

- **[Pixi Development Guide](PIXI_DEVELOPMENT.md)** - Modern package management for local development
- **[Claude Code Configuration](CLAUDE.md)** - AI assistant configuration and workflows
- **[Improvement Plan](IMPROVEMENT_PLAN.md)** - Roadmap and planned enhancements

# Project Structure

```
spanny3/
├── src/                 # Source code
│   ├── rrt.cpp         # RRT algorithm implementation
│   └── rrt_cli.cpp     # Command-line interface
├── include/            # Headers
│   └── spanny/         # Project headers
├── test/               # Tests
│   └── test_rrt.cpp    # RRT algorithm tests
├── config/             # Configuration files
│   └── scenario.json   # Example planning scenario
├── pixi.toml           # Pixi package configuration
├── CMakeLists.txt      # CMake build configuration
├── CMakePresets.json   # CMake presets for different builds
└── compose.dev.yml     # Docker development environment
```

# Contributing

1. Choose your development environment ([Pixi](PIXI_DEVELOPMENT.md) or Docker)
2. Make your changes
3. Run tests: `pixi run test` or `ctest --test-dir build`
4. Submit a pull request

# License

See LICENSE file for details.
