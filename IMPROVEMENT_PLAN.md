# Spanny3 Improvement Plan

## Recent Accomplishments

### 2024-12-26
✅ **Google Benchmark Integration**
- Created comprehensive benchmark suite for RRT algorithm performance analysis
- Integrated Google Benchmark v1.9.1 via CMake FetchContent
- Added Pixi tasks for benchmark execution
- Extended CI/CD pipeline with benchmark job
- Added extensive Doxygen documentation to benchmark code
- Used snake_case naming convention consistent with codebase style

### 2024-11-26
✅ **Pixi Package Management Integration**
- Created comprehensive `pixi.toml` configuration
- Added CMakePresets.json for Pixi builds
- Created detailed PIXI_DEVELOPMENT.md guide
- Updated all documentation to include both Pixi and Docker workflows
- Fixed Docker path inconsistencies across all documentation

## Project Overview ⭐⭐⭐⭐

Spanny3 is a **well-architected C++23 robotics project** showcasing modern C++ practices. The RRT path planning implementation demonstrates clean separation of concerns, excellent testing strategy, and robust infrastructure.

### Strengths
- **Modern C++23**: Excellent use of `std::expected`, concepts, ranges, and designated initializers
- **Clean Architecture**: Well-separated geometric primitives, planning context, and sampling strategies
- **Testability**: Dependency injection enables deterministic testing of stochastic algorithms
- **Infrastructure**: Comprehensive Docker development environment and CI/CD pipeline
- **Coverage**: Well-integrated LLVM coverage reporting with proper filtering

## Code Improvements

### 1. Critical Bug Fix - Edge Relationship (`src/rrt.cpp:118`)

**Priority: HIGH**

```cpp
// Current: Reversed edge relationship
tree.edges.emplace_back(sample.id, closest.id, cost);

// Should be: parent -> child
tree.edges.emplace_back(closest.id, sample.id, cost);
```

**Impact**: This bug breaks tree traversal for path extraction.

### 2. Add Path Extraction Functionality

**Priority: HIGH**

Missing critical functionality to extract actual path from tree when goal is reached.

**Implementation Plan**:
- Add `extract_path(const tree_t& tree, node_id_t goal_id)` function
- Return `std::vector<position_t>` representing the path
- Add comprehensive tests for path extraction

### 3. CLI Improvements (`src/rrt_cli.cpp:37`)

**Priority: MEDIUM**

```cpp
// Current: Hardcoded container path
"src/spanny3/config/scenario.json"

// Should be: Container-relative path
"config/scenario.json"
```

**Additional CLI Enhancements**:
- Add verbose output option
- Add seed parameter for reproducible runs
- Add output format options (JSON, CSV)

### 4. Documentation Enhancements

**Priority: MEDIUM**

- Add algorithm complexity analysis (O(n log n) expected)
- Document collision detection mathematics (quadratic equation solving)
- Include more usage examples and scenarios
- Add architectural decision records (ADRs)

### 5. Docusaurus Documentation Site

**Priority: MEDIUM**

Create comprehensive documentation website using Docusaurus:

**Implementation Steps**:
1. **Initialize Docusaurus**:
   ```bash
   npx create-docusaurus@latest docs classic --typescript
   ```

2. **Documentation Structure**:
   ```
   docs/
   ├── getting-started/
   │   ├── installation.md
   │   ├── docker-setup.md
   │   └── first-run.md
   ├── algorithm/
   │   ├── rrt-overview.md
   │   ├── collision-detection.md
   │   └── performance-analysis.md
   ├── api/
   │   ├── core-types.md
   │   ├── planning-context.md
   │   └── testing-utilities.md
   ├── development/
   │   ├── docker-workflow.md
   │   ├── testing-guide.md
   │   └── contributing.md
   └── examples/
       ├── basic-planning.md
       ├── custom-obstacles.md
       └── visualization.md
   ```

3. **Features to Include**:
   - Interactive algorithm visualization
   - Code examples with syntax highlighting
   - API documentation auto-generated from code
   - Performance benchmarks and charts
   - Docker workflow tutorials
   - Contribution guidelines

4. **Integration**:
   - Deploy to GitHub Pages via CI/CD
   - Auto-update docs from code comments
   - Link to coverage reports and benchmarks

### 6. Error Handling Improvements

**Priority: MEDIUM**

- Add more descriptive error messages with context
- Validate input parameters (positive distances, valid bounds)
- Add error recovery strategies

## Infrastructure Improvements

### 1. Security Enhancements

**Priority: HIGH**

```yaml
# Add to .github/workflows/ci.yml
security:
  name: Security Analysis
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - name: Initialize CodeQL
      uses: github/codeql-action/init@v2
      with:
        languages: cpp
    - name: Autobuild
      uses: github/codeql-action/autobuild@v2
    - name: Perform CodeQL Analysis
      uses: github/codeql-action/analyze@v2
```

### 2. Dependency Updates

**Priority: HIGH**

- Update `actions/checkout@v3` → `@v4` across all workflows
- Update `docker/setup-buildx-action@v2` → `@v3`
- Update `docker/login-action@v2` → `@v3`
- Update `docker/build-push-action@v4` → `@v5`
- Add Dependabot configuration for automated updates

### 3. Performance Testing

**Priority: MEDIUM**

Add benchmark suite:
- Algorithm performance across different scenario sizes
- Memory usage profiling
- Convergence rate analysis
- Add performance regression detection in CI

### 4. CI Matrix Optimization

**Priority: LOW**

Current matrix could be expanded:
```yaml
config:
  - { name: Debug }
  - { name: Release }
  - { name: RelWithDebInfo }  # Add for profiling
  - { name: MinSizeRel }      # Add for size optimization
```

### 6. Pixi Package Management

**Priority: HIGH** *(Elevated from MEDIUM due to development workflow benefits)*

Convert to pixi-based package management for better cross-platform dependency management and simplified development workflow.

#### Detailed Migration Plan

##### Phase 1: Initial Setup (Week 1)

1. **Install Pixi**:
   ```bash
   # Install via official installer (recommended)
   curl -fsSL https://pixi.sh/install.sh | bash

   # Or via Homebrew (macOS/Linux)
   brew install pixi
   ```

2. **Create `pixi.toml` Configuration**:
   ```toml
   [project]
   name = "spanny3"
   version = "0.1.0"
   description = "C++23 RRT path planning implementation with modern robotics algorithms"
   authors = ["griswaldbrooks <griswald.brooks@gmail.com>"]
   channels = ["conda-forge"]
   platforms = ["linux-64", "osx-arm64", "osx-64", "win-64"]

   [environments]
   # Default environment for development
   default = { features = ["dev", "test", "coverage", "docs"], solve-group = "default" }
   # Minimal environment for building only
   build = { features = [], solve-group = "build" }
   # CI environment matching GitHub Actions
   ci = { features = ["test", "coverage"], solve-group = "ci" }

   [dependencies]
   # Core build dependencies
   cmake = ">=3.28,<4"
   ninja = ">=1.11,<2"
   pkg-config = ">=0.29,<1"

   # Clang toolchain (pinned for consistency)
   clangxx = "18.*"
   clang-tools = "18.*"
   llvmdev = "18.*"
   libcxx = "18.*"

   # Third-party libraries (header-only are handled by CMake)
   nlohmann_json = ">=3.11,<4"

   [feature.dev.dependencies]
   # Development tools
   gdb = ">=13,<15"
   lldb = ">=18,<19"
   ccache = ">=4.8,<5"
   pre-commit = ">=3.5,<4"
   codespell = ">=2.2,<3"

   [feature.test.dependencies]
   # Testing framework (will be fetched by CMake but this ensures tools are available)
   gtest = ">=1.14,<2"

   [feature.coverage.dependencies]
   # Coverage tools (part of LLVM)
   lcov = ">=2.0,<3"

   [feature.docs.dependencies]
   # Documentation generation
   doxygen = ">=1.9,<2"
   graphviz = ">=8,<10"
   nodejs = ">=20,<22"  # For Docusaurus

   [feature.lint.dependencies]
   # Linting and formatting
   clang-format = "18.*"
   clang-tidy = "18.*"
   cppcheck = ">=2.12,<3"

   [tasks]
   # Setup and configuration tasks
   configure = { cmd = "cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug", description = "Configure CMake build (Debug)" }
   configure-release = { cmd = "cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release", description = "Configure CMake build (Release)" }
   configure-coverage = { cmd = "cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Coverage", description = "Configure CMake build (Coverage)" }

   # Build tasks
   build = { cmd = "cmake --build build", description = "Build the project", depends-on = ["configure"] }
   build-release = { cmd = "cmake --build build", description = "Build release version", depends-on = ["configure-release"] }
   clean = { cmd = "cmake --build build --target clean", description = "Clean build artifacts" }
   rebuild = { cmd = "cmake --build build --clean-first", description = "Clean and rebuild", depends-on = ["configure"] }

   # Test tasks
   test = { cmd = "ctest --test-dir build --output-on-failure", description = "Run tests", depends-on = ["build"] }
   test-verbose = { cmd = "ctest --test-dir build --output-on-failure --verbose", description = "Run tests with verbose output", depends-on = ["build"] }

   # Coverage tasks
   coverage-build = { cmd = "cmake --build build", description = "Build with coverage", depends-on = ["configure-coverage"] }
   coverage-run = { cmd = "cmake --build build --target coverage", description = "Run full coverage analysis", depends-on = ["coverage-build"] }
   coverage-report = { cmd = "cmake --build build --target coverage-report", description = "Generate coverage report only" }
   coverage-clean = { cmd = "cmake --build build --target coverage-clean", description = "Clean coverage data" }
   coverage = { depends-on = ["coverage-run"], description = "Shorthand for coverage-run" }

   # Code quality tasks
   format = { cmd = "pre-commit run clang-format --all-files", description = "Format code with clang-format" }
   lint = { cmd = "pre-commit run --all-files", description = "Run all pre-commit checks" }
   tidy = { cmd = "run-clang-tidy -p build", description = "Run clang-tidy static analysis" }

   # Development workflow tasks
   dev = { depends-on = ["configure", "build", "test"], description = "Full development cycle" }
   dev-coverage = { depends-on = ["configure-coverage", "coverage-run"], description = "Development with coverage" }
   ci = { depends-on = ["lint", "build", "test"], description = "CI pipeline tasks" }

   # Utility tasks
   run = { cmd = "build/rrt_cli", description = "Run the RRT CLI tool", depends-on = ["build"] }
   run-scenario = { cmd = "build/rrt_cli --config config/scenario.json", description = "Run with default scenario", depends-on = ["build"] }
   shell = { cmd = "bash", description = "Start interactive shell in Pixi environment" }

   [activation]
   # Environment variables
   env = { CXXFLAGS = "-stdlib=libc++", CC = "clang", CXX = "clang++" }

   # Scripts to run on activation
   scripts = ["scripts/setup_env.sh"]
   ```

3. **Create Environment Setup Script** (`scripts/setup_env.sh`):
   ```bash
   #!/usr/bin/env bash
   echo "🚀 Spanny3 Development Environment Activated"
   echo "Clang version: $(clang++ --version | head -n1)"
   echo "CMake version: $(cmake --version | head -n1)"
   echo "Available tasks: pixi task list"
   ```

##### Phase 2: Migration Implementation (Week 1-2)

1. **CMake Adjustments**:
   - Add Pixi-aware CMake presets
   - Ensure CMake finds Pixi-provided dependencies
   - Update compiler detection for Pixi environments

2. **Create `CMakePresets.json`**:
   ```json
   {
     "version": 6,
     "cmakeMinimumRequired": {
       "major": 3,
       "minor": 28,
       "patch": 0
     },
     "configurePresets": [
       {
         "name": "pixi-base",
         "hidden": true,
         "generator": "Ninja",
         "binaryDir": "${sourceDir}/build",
         "cacheVariables": {
           "CMAKE_CXX_COMPILER": "clang++",
           "CMAKE_C_COMPILER": "clang",
           "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
         }
       },
       {
         "name": "pixi-debug",
         "inherits": "pixi-base",
         "displayName": "Pixi Debug",
         "description": "Debug build using Pixi environment",
         "cacheVariables": {
           "CMAKE_BUILD_TYPE": "Debug"
         }
       },
       {
         "name": "pixi-release",
         "inherits": "pixi-base",
         "displayName": "Pixi Release",
         "description": "Release build using Pixi environment",
         "cacheVariables": {
           "CMAKE_BUILD_TYPE": "Release"
         }
       },
       {
         "name": "pixi-coverage",
         "inherits": "pixi-base",
         "displayName": "Pixi Coverage",
         "description": "Coverage build using Pixi environment",
         "cacheVariables": {
           "CMAKE_BUILD_TYPE": "Coverage"
         }
       }
     ],
     "buildPresets": [
       {
         "name": "pixi-debug",
         "configurePreset": "pixi-debug"
       },
       {
         "name": "pixi-release",
         "configurePreset": "pixi-release"
       },
       {
         "name": "pixi-coverage",
         "configurePreset": "pixi-coverage"
       }
     ],
     "testPresets": [
       {
         "name": "pixi-test",
         "configurePreset": "pixi-debug",
         "output": {
           "outputOnFailure": true
         }
       }
     ]
   }
   ```

3. **Update `.gitignore`**:
   ```gitignore
   # Pixi
   .pixi/
   pixi.lock
   ```

##### Phase 3: Workflow Integration (Week 2)

1. **Parallel Workflow Support**:
   - Keep Docker workflow intact for CI/CD
   - Use Pixi for local development
   - Document both workflows clearly

2. **CI/CD Integration**:
   ```yaml
   # .github/workflows/ci-pixi.yml
   name: CI with Pixi
   on: [push, pull_request]

   jobs:
     test:
       strategy:
         matrix:
           os: [ubuntu-latest, macos-latest, windows-latest]
       runs-on: ${{ matrix.os }}
       steps:
         - uses: actions/checkout@v4
         - uses: prefix-dev/setup-pixi@v0.8.0
           with:
             pixi-version: latest
             cache: true
         - run: pixi run ci
   ```

3. **VSCode Integration** (`.vscode/settings.json`):
   ```json
   {
     "cmake.configureSettings": {
       "CMAKE_CXX_COMPILER": "${workspaceFolder}/.pixi/envs/default/bin/clang++",
       "CMAKE_C_COMPILER": "${workspaceFolder}/.pixi/envs/default/bin/clang"
     },
     "cmake.generator": "Ninja",
     "C_Cpp.default.compilerPath": "${workspaceFolder}/.pixi/envs/default/bin/clang++",
     "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
   }
   ```

##### Phase 4: Documentation and Training (Week 2-3)

1. **Update README.md** with Pixi quickstart
2. **Update CLAUDE.md** with Pixi commands
3. **Create migration guide** for team members
4. **Add troubleshooting section**

#### Benefits of Pixi Migration

1. **Simplified Onboarding**: Single command setup: `pixi install && pixi run dev`
2. **Cross-Platform Support**: Works on Linux, macOS, and Windows without Docker
3. **Faster Iteration**: No container overhead for local development
4. **Dependency Locking**: `pixi.lock` ensures reproducible environments
5. **Task Automation**: Built-in task runner replaces Makefiles/scripts
6. **IDE Integration**: Better support for VSCode, CLion, etc.
7. **CI/CD Flexibility**: Can run in containers or native environments

#### Migration Strategy

1. **Phase 1 (Immediate)** ✅ **COMPLETED** (2025-09-26):
   - Created `pixi.toml` with full configuration and platform-specific dependencies
   - Created CMakePresets.json for Pixi builds
   - Created comprehensive documentation (PIXI_DEVELOPMENT.md)
   - Updated CLAUDE.md and README.md with Pixi workflows
   - Fixed Linux-specific dependencies (libcxx, libcxxabi, libcxx-devel, compiler-rt)
   - Fixed macOS compatibility (platform-specific debuggers)
   - Removed Windows support (libc++ not available in conda-forge)
   - Tested all 23 pixi tasks successfully on Linux

2. **Phase 2 (Week 1)** - **READY FOR TESTING**:
   - ✅ Tested on Linux platform
   - ⏳ Test on macOS (ARM64 & x64)
   - ⏳ Test CI/CD integration
   - ⏳ Gather team feedback

3. **Phase 3 (Week 2)**:
   - Team training and onboarding
   - Gather feedback and iterate
   - Performance comparison with Docker workflow

4. **Phase 4 (Week 3)**:
   - Finalize workflow
   - Update all documentation
   - Consider deprecating Docker workflow if Pixi proves superior

#### Success Metrics

- **Setup Time**: ✅ < 5 minutes from clone to running tests (achieved on Linux)
- **Platform Coverage**: 🟡 Works on Linux and macOS (ARM64 & x64); Windows not supported
- **CI Performance**: ⏳ Equal or better than Docker-based CI (pending testing)
- **Developer Satisfaction**: Positive feedback from team

#### Rollback Plan

If Pixi adoption faces issues:
1. Docker workflow remains fully functional
2. Can run both workflows in parallel indefinitely
3. Pixi files can be removed without affecting Docker setup

### 7. Static Analysis Enhancement

**Priority: MEDIUM**

Add clang-tidy integration for advanced static analysis:

**Implementation Steps**:
1. **Create `.clang-tidy` configuration**:
   ```yaml
   Checks: >
     readability-*,
     performance-*,
     modernize-*,
     bugprone-*,
     clang-analyzer-*,
     cppcoreguidelines-*,
     -modernize-use-trailing-return-type
   ```

2. **Integrate with build system**:
   ```cmake
   option(ENABLE_CLANG_TIDY "Enable clang-tidy analysis" OFF)
   if(ENABLE_CLANG_TIDY)
     find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
     set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE})
   endif()
   ```

3. **Add to pre-commit hooks and CI/CD**

## Architecture Improvements

### 1. Plugin System for RRT Variants

**Priority: MEDIUM**

Extract sampling strategies into pluggable interfaces:
```cpp
template<typename SamplingStrategy>
class rrt_planner_t {
    SamplingStrategy strategy_;
    // ...
};

// Variants: RRT*, Bi-RRT, RRT-Connect
```

### 2. Visualization Support

**Priority: LOW**

Add optional visualization output:
- SVG export for web viewing
- JSON format for custom visualizers
- Real-time plotting integration

### 3. Configuration Validation

**Priority: MEDIUM**

- Add JSON schema for scenario files
- Runtime validation with descriptive errors
- Configuration file documentation generation

### 4. Memory Optimization

**Priority: LOW**

For large-scale scenarios:
- Consider spatial data structures (KD-tree) for nearest neighbor queries
- Memory pool allocators for frequent node allocation/deallocation
- Incremental tree building with pruning

## Testing Improvements

### 1. Expanded Test Coverage

**Priority: MEDIUM**

- Add property-based testing for geometric operations
- Stress testing with complex obstacle configurations
- Edge case testing (degenerate scenarios)

### 2. Integration Testing

**Priority: MEDIUM**

- End-to-end CLI testing with various scenarios
- Docker container integration tests
- Cross-platform compatibility tests

### 3. Benchmark Integration ✅ **COMPLETED** (2024-12-26)

**Priority: LOW**

- ✅ Added Google Benchmark integration via CMake FetchContent
- ✅ Created comprehensive benchmark suite (`benchmark/benchmark_rrt.cpp`):
  - RRT planning benchmarks (simple and with obstacles)
  - Collision detection scaling tests
  - Nearest neighbor search performance characterization
- ✅ Integrated benchmarks into Pixi workflow (`pixi run benchmark`)
- ✅ Added CI/CD support with GitHub Actions
- ✅ Results uploaded as JSON artifacts and displayed in step summary
- 🔲 Performance regression detection (future enhancement)
- 🔲 Memory usage benchmarks (future enhancement)

## Docker Workflow Analysis & Improvements

### Current Issues

1. **Manual UID/GID Export**: Requires users to manually export environment variables
2. **Privileged Mode**: Using `privileged: true` is overly broad security risk
3. **Host Network**: `network_mode: host` reduces container isolation
4. **No Persistent Caches**: Build artifacts and caches are recreated each time

### Docker-Native Solutions

**Note**: Docker has no truly native way to automatically detect host UID/GID. The manual export approach is the cleanest Docker-native solution.

#### 1. UID/GID Handling

**Keep manual export but document clearly:**
```bash
# Required before running development container
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml run --rm development
```

#### 2. Enhanced Security

```yaml
# compose.improved.yml - Reduced privileges
privileged: false
cap_add:
  - SYS_PTRACE  # Only for debugging
security_opt:
  - seccomp:unconfined  # Only if debugging tools require
```

#### 3. Better Networking

```yaml
# Custom bridge network instead of host mode
networks:
  - spanny-dev
ports:
  - "5051:5051"  # Specific port mapping for LSP
```

#### 4. Persistent Volume Management

```yaml
volumes:
  # Performance: persistent build caches
  - spanny3-artifacts:/home/${USER}/ws/artifacts
  - spanny3-ccache:/home/${USER}/.ccache
  - spanny3-pre-commit:/home/${USER}/.cache/pre-commit
```

## Implementation Priority

### Phase 1 (Critical - Week 1)
- [ ] Fix edge relationship bug in `src/rrt.cpp:118`
- [ ] Docker workflow improvements (security, caching)
- [ ] Dependency updates (GitHub Actions)
- [ ] Security scanning integration

### Phase 2 (High Priority - Week 2-3)
- [ ] Path extraction functionality
- [ ] CLI improvements (container-relative paths)
- [ ] clang-tidy integration and configuration
- [ ] Expanded test coverage

### Phase 2.5 (High Priority - Week 2)
- [x] Pixi package management setup (COMPLETED)
- [x] CMake presets configuration for Pixi (COMPLETED)
- [x] Update documentation with Pixi workflows (COMPLETED)

### Phase 3 (Medium Priority - Month 1)
- [ ] Performance testing suite
- [ ] Documentation enhancements
- [ ] Configuration validation with JSON schema
- [ ] Docusaurus documentation site setup

### Phase 4 (Enhancement - Month 2+)
- [ ] Plugin system for RRT variants
- [ ] Visualization support (SVG/JSON export)
- [ ] Advanced static analysis
- [ ] Cross-platform testing and CI

## Success Metrics

- **Code Quality**: Maintain >95% test coverage
- **Performance**: Sub-second planning for typical scenarios
- **Usability**: One-command development environment setup
- **Security**: Zero high-severity vulnerabilities
- **Documentation**: Complete API coverage and examples

---

*This improvement plan should be reviewed and prioritized based on project goals and resource availability.*
