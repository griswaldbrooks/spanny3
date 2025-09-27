# Pixi Development Setup

Learn how to use Pixi for efficient cross-platform C++ development.

## What is Pixi?

Pixi is a modern package manager for conda packages that provides:
- **Fast dependency resolution** with lock files
- **Cross-platform consistency** (Linux and macOS)
- **Task automation** built into the tool
- **No container overhead** for local development

## Common Commands

### Development Cycle

```bash
# Complete cycle: configure, build, test
pixi run dev

# Individual steps
pixi run build        # Build only
pixi run test         # Run tests
pixi run lint         # Run code quality checks
```

### Performance Analysis

```bash
# Run benchmarks
pixi run benchmark

# Save benchmark results to JSON
pixi run benchmark-json

# View results
cat build/benchmark_results.json
```

### Coverage Analysis

```bash
# Generate coverage report
pixi run coverage

# View HTML report
open build/coverage/reports/html/index.html
```

### Running Examples

```bash
# Run with default scenario
pixi run run-scenario

# Run with custom scenario
pixi run build
./build/rrt_cli --scenario path/to/scenario.json
```

## Pixi Environment

### Activating the Shell

For interactive development, activate the Pixi shell:

```bash
pixi shell
```

This gives you direct access to all tools:
```bash
clang++ --version
cmake --version
```

### Available Tasks

See all configured tasks:
```bash
pixi task list
```

### Environment Information

Check your environment setup:
```bash
pixi info
```

## Project Structure

```
.pixi/                  # Managed by Pixi (git-ignored)
├── envs/
│   └── default/       # Development environment
pixi.toml              # Package and task configuration
pixi.lock              # Lock file (committed to git)
```

## Configuration Files

### pixi.toml

Defines dependencies and tasks:
- **Dependencies**: Clang, CMake, GoogleTest, etc.
- **Tasks**: build, test, coverage, benchmark
- **Platform-specific**: Linux requires libcxx packages

### pixi.lock

Lock file ensuring reproducible builds:
- ✅ **Always commit** after `pixi install`
- ✅ Used by CI/CD for consistency
- ✅ Guarantees same versions across machines

## CMake Presets

Pixi-specific CMake presets are available:

```bash
# Using presets directly
cmake --preset pixi-debug
cmake --build --preset pixi-debug
ctest --preset pixi-test
```

## CI/CD Integration

The same Pixi commands work in CI:

```yaml
# .github/workflows/ci.yml
- uses: prefix-dev/setup-pixi@v0.8.1
- run: pixi run dev
```

## Next Steps

- [First Run](first-run.md) - Run your first planning scenario
- [Development Workflow](../development/pixi-workflow.md) - Advanced workflows
- [Contributing](../development/contributing.md) - Contribution guidelines
