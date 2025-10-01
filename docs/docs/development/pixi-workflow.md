---
sidebar_position: 1
---

# Advanced Pixi Workflows

Advanced techniques for customizing and optimizing your Pixi development environment.

## Custom Tasks

Create project-specific tasks to streamline your workflow.

### Adding a Task

Edit `pixi.toml` to add custom tasks:

```toml
[tasks]
my-task = { cmd = "echo 'Building custom target'", description = "My custom task" }
quick-check = { cmd = "clang++ -std=c++23 -fsyntax-only src/rrt.cpp", description = "Quick syntax check" }
```

Run with:
```bash
pixi run my-task
```

### Task Dependencies

Chain tasks together using `depends-on`:

```toml
[tasks]
configure-debug = { cmd = "cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug", description = "Configure debug build" }
build-debug = { cmd = "cmake --build build", description = "Build debug", depends-on = ["configure-debug"] }
```

When you run `pixi run build-debug`, it automatically runs `configure-debug` first.

### Task Composition

The project uses task composition extensively:

```toml
[tasks]
dev = { depends-on = ["configure", "build", "test"], description = "Full development cycle" }
```

This runs three tasks in sequence with a single command: `pixi run dev`

### Conditional Tasks

Use shell logic for platform-specific behavior:

```bash
[tasks]
platform-info = { cmd = "uname -s || echo 'Unknown platform'", description = "Show platform" }
```

## Environment Management

### Multiple Environments

Pixi supports separate environments for different workflows:

```toml
[environments]
default = { features = ["dev", "test", "benchmark"], solve-group = "default" }
build = { features = [], solve-group = "build" }
ci = { features = ["test", "benchmark", "coverage"], solve-group = "ci" }
```

Use specific environments:

```bash
# Use minimal build environment
pixi run -e build build

# Use CI environment
pixi run -e ci test
```

### Feature Sets

Features group related dependencies:

```toml
[feature.test.dependencies]
gtest = ">=1.14,<2"

[feature.benchmark.dependencies]
# Benchmark dependencies

[feature.coverage.dependencies]
# Coverage-specific tools
```

Activate features selectively:

```bash
# Default environment includes all features
pixi install

# Only install build dependencies
pixi install -e build
```

### Environment Variables

Set environment variables for tasks:

```toml
[activation.env]
CC = "clang"
CXX = "clang++"

[target.linux-64.activation.env]
CXXFLAGS = "-stdlib=libc++"
```

These are automatically set when running `pixi run` commands.

## IDE Integration

### VSCode Setup

The project includes `.vscode/settings.json` with Pixi integration:

```json
{
  "cmake.configureOnOpen": true,
  "cmake.cmakePath": "${workspaceFolder}/.pixi/envs/default/bin/cmake",
  "C_Cpp.default.compilerPath": "${workspaceFolder}/.pixi/envs/default/bin/clang++"
}
```

For manual setup:

1. Install C/C++ and CMake Tools extensions
2. Open Command Palette (Ctrl+Shift+P)
3. Select "CMake: Select a Kit"
4. Choose "Clang 18" from Pixi environment

### Using Pixi Shell with IDEs

For IDEs that need environment activation:

```bash
# Enter Pixi environment
pixi shell

# Launch your IDE from the shell
code .        # VSCode
clion .       # CLion
```

The IDE inherits Pixi's environment variables and paths.

### CLion Configuration

1. Go to Settings → Build, Execution, Deployment → CMake
2. Add CMake options: `-DCMAKE_CXX_COMPILER=clang++`
3. Set environment variables:
   ```
   CC=clang
   CXX=clang++
   PATH=.pixi/envs/default/bin:$PATH
   ```

### Debugging with Pixi

Use platform-specific debuggers provided by Pixi:

```bash
# Linux
pixi run -e dev gdb build/rrt_cli

# macOS
pixi run -e dev lldb build/rrt_cli
```

Or configure your IDE to use Pixi's debugger:
- Linux: `.pixi/envs/default/bin/gdb`
- macOS: `.pixi/envs/default/bin/lldb`

## Dependency Management

### Adding Dependencies

Add conda-forge packages to `pixi.toml`:

```toml
[dependencies]
boost = ">=1.80,<2"
```

Then update:

```bash
pixi install
git add pixi.lock
git commit -m "Add Boost dependency"
```

**Important**: Always commit `pixi.lock` to ensure reproducible builds.

### Version Constraints

Use semantic versioning for dependencies:

```toml
[dependencies]
cmake = ">=3.28,<4"      # Major version constraint
clangxx = "18.*"          # Exact minor version
nlohmann_json = ">=3.11,<4"  # Range
```

### Platform-Specific Dependencies

Add dependencies for specific platforms:

```toml
[target.linux-64.dependencies]
libcxx = "18.*"
gdb = ">=13,<15"

[target.osx-arm64.dependencies]
lldb = ">=18,<19"
```

### Updating Dependencies

```bash
# Update all dependencies to latest matching versions
pixi update

# Update specific package
pixi update cmake

# Check for available updates
pixi list
```

## Troubleshooting

### Command Not Found Errors

**Problem**: `pixi: command not found`

**Solution**: Reload shell configuration:
```bash
source ~/.bashrc  # or ~/.zshrc
```

Or add to your PATH manually:
```bash
export PATH="$HOME/.pixi/bin:$PATH"
```

### Build Failures

**Problem**: Build fails with "compiler not found"

**Solution**: Use `pixi run` prefix:
```bash
# Wrong - uses system compiler
cmake --build build

# Correct - uses Pixi compiler
pixi run build
```

### Dependency Conflicts

**Problem**: Package version conflicts during install

**Solution**:
1. Check `pixi.toml` for conflicting version constraints
2. Try updating lock file:
   ```bash
   rm pixi.lock
   pixi install
   ```
3. Use solve groups to isolate environments

### Linker Errors on Linux

**Problem**: Cannot find libc++ libraries

**Solution**: Ensure you're using Pixi environment:
```bash
# Check if using correct libraries
pixi shell
echo $LD_LIBRARY_PATH  # Should include .pixi/envs/default/lib
```

The CMake configuration already sets RPATH:
```cmake
-DCMAKE_EXE_LINKER_FLAGS='-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib'
```

### Cache Issues

**Problem**: Stale build artifacts or dependencies

**Solution**: Clean and rebuild:
```bash
# Clean CMake build
pixi run clean
rm -rf build/

# Reinstall Pixi environment
rm -rf .pixi/
pixi install
```

### Task Execution Failures

**Problem**: Task fails with cryptic error

**Solution**: Run with verbose output:
```bash
# Check task definition
pixi task list

# Run task manually to see full output
pixi shell
cd /path/to/project
# Copy task command from pixi.toml and run it
```

## Performance Optimization

### Parallel Builds

Enable parallel compilation in CMake:

```toml
[tasks]
build = { cmd = "cmake --build build -j8", description = "Build with 8 threads" }
```

Or use all available cores:
```bash
cmake --build build -j$(nproc)  # Linux
cmake --build build -j$(sysctl -n hw.ncpu)  # macOS
```

### Ccache Integration

The project includes ccache for faster rebuilds:

```toml
[feature.dev.dependencies]
ccache = ">=4.8,<5"
```

CMake automatically detects and uses ccache when available.

### Incremental Builds

After initial build, use incremental builds:

```bash
# Only rebuild changed files
pixi run build

# Full clean rebuild when needed
pixi run rebuild
```

## Advanced Use Cases

### CI/CD Integration

Use Pixi in GitHub Actions:

```yaml
- uses: prefix-dev/setup-pixi@v0.4.1
  with:
    pixi-version: v0.25.0

- name: Build
  run: pixi run build

- name: Test
  run: pixi run test
```

This ensures identical environments locally and in CI.

### Cross-Platform Development

Pixi handles platform differences automatically:

```toml
[target.linux-64.dependencies]
libcxx = "18.*"  # Only on Linux

[target.osx-arm64.dependencies]
# macOS uses system libc++
```

Same commands work everywhere:
```bash
pixi run dev  # Works on Linux and macOS
```

### Custom CMake Presets

Create additional CMake presets for specific workflows:

```json
{
  "configurePresets": [
    {
      "name": "pixi-asan",
      "inherits": "pixi-debug",
      "cacheVariables": {
        "CMAKE_CXX_FLAGS": "-fsanitize=address"
      }
    }
  ]
}
```

Use with:
```bash
pixi shell
cmake --preset pixi-asan
cmake --build --preset pixi-asan
```

### Sharing Environments

Commit `pixi.lock` to share exact dependency versions:

```bash
git add pixi.lock
git commit -m "Update dependencies"
git push
```

Team members get identical environments:
```bash
git pull
pixi install  # Uses committed pixi.lock
```

## Best Practices

1. **Always use `pixi run`**: Ensures correct environment activation
2. **Commit `pixi.lock`**: Guarantees reproducible builds across machines
3. **Use features**: Organize dependencies by purpose (dev, test, benchmark)
4. **Leverage task composition**: Create high-level workflows from simple tasks
5. **Platform-specific configs**: Use `[target.*.dependencies]` for platform needs
6. **Version constraints**: Specify ranges to allow updates while preventing breakage

## See Also

- [Getting Started: Pixi Setup](../getting-started/pixi-setup.md) - Basic Pixi introduction
- [Testing Guide](testing-guide.md) - Running and writing tests
- [Contributing Guide](contributing.md) - Development workflow for contributors
- [Pixi Documentation](https://pixi.sh/docs) - Official Pixi documentation
