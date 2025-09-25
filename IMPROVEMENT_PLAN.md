# Spanny3 Improvement Plan

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

### 5. Error Handling Improvements

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

### 5. Pixi Package Management

**Priority: MEDIUM**

Convert to pixi-based package management for better dependency management:

**Implementation Steps**:
1. **Create `pixi.toml`**:
   ```toml
   [project]
   name = "spanny3"
   description = "C++23 RRT path planning implementation"
   authors = ["griswaldbrooks <email@example.com>"]

   [environments]
   default = { solve-group = "default" }

   [dependencies]
   cmake = ">=3.28"
   clang = ">=18"
   ninja = ">=1.11"
   clang-tools = ">=18"  # Includes clang-tidy, clang-format

   [feature.test.dependencies]
   gtest = ">=1.14"
   gmock = ">=1.14"

   [feature.coverage.dependencies]
   llvm-tools = ">=18"  # For llvm-cov, llvm-profdata

   [tasks]
   build = "cmake -S . -B build && cmake --build build"
   test = "ctest --test-dir build --output-on-failure"
   coverage = "cmake -DCMAKE_BUILD_TYPE=Coverage -S . -B build && cmake --build build --target coverage"
   lint = "pre-commit run --all-files"
   tidy = "cmake --build build --target clang-tidy"
   ```

2. **Benefits**:
   - Cross-platform dependency management
   - Reproducible development environments
   - Task automation without scripts
   - Integration with conda-forge ecosystem

3. **Migration Strategy**:
   - Keep Docker workflow for CI/CD consistency
   - Use pixi for local development
   - Document both approaches in README

### 6. Static Analysis Enhancement

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

### 3. Benchmark Integration

**Priority: LOW**

- Add Google Benchmark integration
- Performance regression detection
- Memory usage benchmarks

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
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.improved.yml run --rm development
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

### Phase 3 (Medium Priority - Month 1)
- [ ] Pixi package management setup
- [ ] Performance testing suite
- [ ] Documentation enhancements
- [ ] Configuration validation with JSON schema

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