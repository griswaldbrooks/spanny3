# Installation

Get started with Spanny3 by setting up your development environment using Pixi.

## Prerequisites

- **Operating System**: Linux or macOS (Windows not currently supported)
- **Git**: For cloning the repository

## Quick Start

### 1. Install Pixi

Pixi is a cross-platform package manager that handles all dependencies for you.

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

After installation, restart your terminal or run:
```bash
source ~/.bashrc  # or ~/.zshrc on macOS
```

### 2. Clone the Repository

```bash
git clone https://github.com/griswaldbrooks/spanny3.git
cd spanny3
```

### 3. Install Dependencies

```bash
pixi install
```

This single command installs:
- Clang 18 compiler with libc++
- CMake and Ninja build tools
- GoogleTest/GoogleMock testing frameworks
- Google Benchmark performance tools
- Development tools (clang-format, codespell, etc.)

### 4. Build and Test

```bash
pixi run dev
```

This runs the complete development cycle:
1. Configures the project with CMake
2. Builds all targets
3. Runs the test suite

## What's Next?

- [Pixi Setup](pixi-setup.md) - Learn about the Pixi development workflow
- [First Run](first-run.md) - Run your first path planning scenario
- [Development Workflow](../development/pixi-workflow.md) - Detailed development guide

## Troubleshooting

### Linux: Missing libc++ Errors

If you see linker errors about missing libc++, ensure you're using the Pixi environment:

```bash
pixi shell  # Activate the environment
```

### macOS: Xcode Command Line Tools

macOS users need Xcode Command Line Tools:

```bash
xcode-select --install
```

## Alternative: Docker (Deprecated)

The Docker workflow has been removed in favor of Pixi. If you need containerized builds, please open an issue on GitHub.
