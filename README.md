[![ci](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml) [![coverage](https://img.shields.io/badge/coverage-report-blue.svg)](https://github.com/griswaldbrooks/spanny3/actions)
# spanny3
Robot arm project for CppCon 2024 presentation.

# Development Environment

## 🚀 [Pixi Development](PIXI_DEVELOPMENT.md)

This project uses **Pixi** for package management and development workflows.

### Benefits
- **Quick setup**: One command installs everything
- **Native performance**: No container overhead
- **Better IDE integration**: Debuggers and tools work naturally
- **Cross-platform**: Works on Linux and macOS (Windows not yet supported)
- **Reproducible**: Lock file ensures consistent environments

### Quick Start

```bash
# Install Pixi (one-time)
curl -fsSL https://pixi.sh/install.sh | bash

# Setup and build
pixi install
pixi run dev
```

[**→ Full Pixi Development Guide**](PIXI_DEVELOPMENT.md)

## Common Development Commands

```bash
# Complete development cycle (configure, build, test)
pixi run dev

# Individual tasks
pixi run build           # Build the project
pixi run test            # Run tests
pixi run coverage        # Generate coverage report
pixi run lint            # Run all linters
pixi run run-scenario    # Run with example scenario

# See all available tasks
pixi task list
```

## Coverage

Generate and view code coverage reports:

```bash
# Run full coverage analysis
pixi run coverage

# View reports
open build/coverage/reports/html/index.html  # HTML report
cat build/coverage/reports/coverage.txt      # Text summary
```

# Documentation

- **[Pixi Development Guide](PIXI_DEVELOPMENT.md)** - Package management and workflows
- **[Claude Code Configuration](CLAUDE.md)** - AI assistant configuration
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
└── CMakePresets.json   # CMake presets for Pixi builds
```

# Contributing

1. Set up your development environment with [Pixi](PIXI_DEVELOPMENT.md)
2. Make your changes
3. Run tests: `pixi run test`
4. Run linting: `pixi run lint`
5. Submit a pull request

# License

See LICENSE file for details.