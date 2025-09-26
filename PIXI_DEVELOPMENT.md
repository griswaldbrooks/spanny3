# Pixi Development Guide

## What is Pixi?

[Pixi](https://pixi.sh) is a modern, cross-platform package manager that makes it easy to manage development dependencies and environments. For C++ projects like Spanny3, it provides a simpler alternative to Docker for local development while maintaining reproducibility.

## Why Use Pixi for Spanny3?

### Benefits Over Manual Setup
- ✅ **One command** installs all dependencies
- ✅ **Consistent versions** across all developers
- ✅ **No system pollution** - everything is project-local
- ✅ **Cross-platform** - works on Linux, macOS, and Windows

### Benefits Over Docker
- ✅ **Native performance** - no container overhead
- ✅ **Better IDE integration** - debuggers and tools work naturally
- ✅ **Simpler commands** - no UID/GID exports needed
- ✅ **Faster iteration** - instant rebuilds without container layers

## Quick Start

### 1. Install Pixi

Choose one of these methods:

```bash
# Official installer (Recommended)
curl -fsSL https://pixi.sh/install.sh | bash

# Or using Homebrew (macOS/Linux)
brew install pixi

# Or using Windows PowerShell
iwr -useb https://pixi.sh/install.ps1 | iex
```

### 2. Clone and Setup Project

```bash
# Clone the repository
git clone https://github.com/griswaldbrooks/spanny3.git
cd spanny3

# Install all dependencies (one-time)
pixi install
```

### 3. Build and Run

```bash
# Complete development cycle (configure, build, test)
pixi run dev

# Or run individual tasks
pixi run build       # Build only
pixi run test        # Test only
pixi run coverage    # Generate coverage report
```

That's it! You're ready to develop.

## Common Development Tasks

### Building the Project

```bash
# Debug build (default)
pixi run build

# Release build (optimized)
pixi run build-release

# Clean and rebuild
pixi run clean
pixi run rebuild
```

### Running Tests

```bash
# Run all tests
pixi run test

# Run tests with verbose output
pixi run test-verbose

# Generate coverage report
pixi run coverage
```

### Code Quality

```bash
# Format code
pixi run format

# Run all linters
pixi run lint

# Run static analysis
pixi run tidy
```

### Running the Application

```bash
# Run with default arguments
pixi run run

# Run with example scenario
pixi run run-scenario
```

## Understanding the Pixi Workflow

### Project Structure

```
spanny3/
├── pixi.toml           # Pixi configuration (like package.json)
├── pixi.lock           # Lock file (like package-lock.json)
├── .pixi/              # Local environment (git-ignored)
│   └── envs/           # Installed tools live here
├── build/              # CMake build directory
└── src/                # Your source code
```

### How Pixi Works

1. **pixi.toml** defines:
   - Required dependencies (Clang, CMake, etc.)
   - Build tasks and commands
   - Environment settings

2. **pixi install** creates:
   - Isolated environment in `.pixi/`
   - Downloads exact versions specified
   - Sets up activation scripts

3. **pixi run** commands:
   - Activate the environment
   - Run with correct paths and settings
   - Use project-local tools only

## Available Tasks

View all available tasks:
```bash
pixi task list
```

Key tasks include:

| Task | Description | Command |
|------|-------------|---------|
| `dev` | Complete development cycle | `pixi run dev` |
| `build` | Build debug version | `pixi run build` |
| `build-release` | Build optimized version | `pixi run build-release` |
| `test` | Run test suite | `pixi run test` |
| `coverage` | Generate coverage report | `pixi run coverage` |
| `lint` | Run all code checks | `pixi run lint` |
| `format` | Auto-format code | `pixi run format` |
| `clean` | Clean build artifacts | `pixi run clean` |
| `shell` | Enter Pixi environment | `pixi shell` |

## IDE Integration

### VSCode

The project includes VSCode settings that automatically detect Pixi:

1. Open the project in VSCode
2. Install the C/C++ and CMake Tools extensions
3. VSCode will automatically use Pixi's compiler

### CLion

1. Open the project in CLion
2. Go to Settings → Build, Execution, Deployment → CMake
3. Set CMake options: `-DCMAKE_CXX_COMPILER=clang++`
4. Set environment variables: `CC=clang CXX=clang++`

### Manual CMake with Pixi

```bash
# Enter Pixi environment
pixi shell

# Now use CMake directly
cmake --preset pixi-debug
cmake --build --preset pixi-debug
ctest --preset pixi-test
```

## Troubleshooting

### Command Not Found

If `pixi` is not found after installation:
```bash
# Reload your shell configuration
source ~/.bashrc  # or ~/.zshrc on macOS
```

### Permission Denied

If you get permission errors:
```bash
# Make sure scripts are executable
chmod +x scripts/setup_env.sh
```

### Build Errors

If builds fail:
```bash
# Clean everything and start fresh
pixi run clean
rm -rf build/
pixi run dev
```

### Dependency Conflicts

If you have system-wide Clang/CMake conflicts:
```bash
# Pixi isolates everything, so use pixi shell
pixi shell
which clang++  # Should show .pixi/envs/default/bin/clang++
```

## Advanced Usage

### Using Different Environments

```bash
# Use minimal build environment
pixi run -e build build

# Use CI environment
pixi run -e ci test
```

### Adding New Dependencies

Edit `pixi.toml`:
```toml
[dependencies]
your-package = ">=1.0"
```

Then update:
```bash
pixi install
```

### Creating New Tasks

Add to `pixi.toml`:
```toml
[tasks]
my-task = { cmd = "echo 'Hello'", description = "My custom task" }
```

Run with:
```bash
pixi run my-task
```

## Pixi vs Docker Comparison

| Aspect | Pixi | Docker |
|--------|------|--------|
| **Installation** | Simple installer | Requires Docker Desktop |
| **Performance** | Native speed | Container overhead |
| **IDE Support** | Excellent | Limited |
| **Commands** | `pixi run build` | Long docker-compose commands |
| **Debugging** | Native debuggers work | Complex setup |
| **CI/CD** | Good | Excellent |
| **Reproducibility** | Very good | Perfect |

## When to Use Docker Instead

While Pixi is great for development, use Docker for:

- **CI/CD pipelines** - GitHub Actions already set up
- **Production deployments** - Container isolation
- **Team members who can't install Pixi** - Fallback option
- **Absolute reproducibility** - Bit-for-bit identical

## Migration from Docker

If you're currently using Docker:

1. **Keep Docker setup** - Don't remove it
2. **Try Pixi locally** - Test the workflow
3. **Compare speed** - Pixi should be faster
4. **Gradually switch** - Use what works best

Both can coexist - Docker for CI/CD, Pixi for development.

## Getting Help

### Resources

- [Pixi Documentation](https://pixi.sh/docs)
- [Pixi GitHub](https://github.com/prefix-dev/pixi)
- [Project README](README.md)
- [Claude Code Configuration](CLAUDE.md)
- [Improvement Plan](IMPROVEMENT_PLAN.md)

### Common Issues

- **Issue**: Clang not found
  - **Solution**: Run commands with `pixi run` prefix

- **Issue**: CMake can't find dependencies
  - **Solution**: Use `pixi shell` or CMake presets

- **Issue**: Different behavior than Docker
  - **Solution**: Check `pixi.toml` matches Dockerfile

### Platform Notes

- **Linux**: Requires `libcxx`, `libcxxabi`, `libcxx-devel`, and `compiler-rt` for C++23 support
- **macOS**: Uses system libc++, no additional runtime libraries needed
- **Windows**: Currently not supported (libc++ not available in conda-forge)

## Next Steps

1. **Try the basics**: `pixi run dev`
2. **Explore tasks**: `pixi task list`
3. **Read the code**: Start with `src/rrt.cpp`
4. **Run examples**: `pixi run run-scenario`
5. **Make changes**: Edit code and `pixi run test`

Welcome to simplified C++ development with Pixi! 🚀