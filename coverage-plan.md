# Code Coverage Implementation Plan

## Overview
Add comprehensive code coverage reporting to the Spanny3 C++ robotics project to track test effectiveness and identify untested code paths.

## Current Environment
- **Compiler**: Clang 18 with libc++
- **Build System**: CMake
- **Testing**: GoogleTest/GoogleMock via CTest
- **Development**: Docker containerized environment
- **CI/CD**: GitHub Actions

## Coverage Tool Selection

### Primary Option: LLVM Coverage (llvm-cov)
- **Pros**: Native Clang integration, excellent C++23 support, built into our existing toolchain
- **Implementation**: Use `-fprofile-instr-generate -fcoverage-mapping` compiler flags
- **Output Formats**: HTML reports, JSON, LCOV format
- **Integration**: Minimal dependencies, works with existing Clang 18 setup

### Alternative: gcov/lcov
- **Pros**: Industry standard, extensive tooling ecosystem
- **Cons**: Requires GCC or additional setup with Clang
- **Decision**: Skip in favor of native LLVM tooling

## Implementation Plan

### Phase 1: CMake Integration
1. **Coverage Build Configuration**
   - Add `CMAKE_BUILD_TYPE=Coverage` option
   - Configure compiler flags: `-fprofile-instr-generate -fcoverage-mapping`
   - Add coverage-specific linker flags
   - Create separate build target for coverage builds

2. **CMake Targets**
   - `coverage-build`: Build with coverage instrumentation
   - `coverage-run`: Execute tests with profile data collection
   - `coverage-report`: Generate HTML and text reports
   - `coverage-clean`: Remove profile data files

### Phase 2: Docker Environment Updates
1. **Development Container**
   - Ensure llvm-cov tools are available in container
   - Add coverage report output directory mounting
   - Update CLAUDE.md with coverage commands

2. **Build Commands**
   - Coverage build: `cmake -DCMAKE_BUILD_TYPE=Coverage`
   - Profile generation: Run tests with `LLVM_PROFILE_FILE` environment variable
   - Report generation: `llvm-cov show` and `llvm-cov report`

### Phase 3: Automated Reporting
1. **Local Development**
   - HTML reports viewable in browser
   - Terminal summary reports
   - Integration with existing test workflow

2. **CI/CD Integration**
   - GitHub Actions workflow for coverage collection
   - Artifact upload for coverage reports
   - Optional: Coverage badge generation
   - Optional: Coverage threshold enforcement

### Phase 4: Report Outputs
1. **HTML Reports**
   - Line-by-line coverage visualization
   - Function and branch coverage metrics
   - File-level coverage summaries
   - Interactive browsing of source code

2. **Summary Reports**
   - Overall project coverage percentage
   - Per-file coverage breakdown
   - Uncovered line identification
   - Branch coverage analysis

## File Structure
```
coverage/
├── reports/
│   ├── html/          # HTML coverage reports
│   ├── coverage.txt   # Text summary
│   └── coverage.json  # Machine-readable data
├── profiles/          # Raw profile data (.profraw files)
└── merged.profdata    # Merged profile database
```

## Integration Points

### CLAUDE.md Updates
Add coverage-specific commands:
- **Coverage Build**: Full build with instrumentation
- **Coverage Test**: Run tests and collect profile data
- **Coverage Report**: Generate and view HTML reports
- **Coverage Clean**: Reset profile data

### Docker Compose
- Mount coverage output directory for report viewing
- Ensure proper permissions for profile data collection
- Volume mapping for persistent coverage data

### GitHub Actions
- Add coverage collection step to existing CI
- Artifact upload for coverage reports
- Optional PR comment with coverage summary

## Success Criteria
1. ✅ Accurate line and branch coverage metrics
2. ✅ Easy-to-use development workflow
3. ✅ HTML reports viewable locally and in CI
4. ✅ Integration with existing build/test commands
5. ✅ Zero impact on production builds
6. ✅ Clear documentation in CLAUDE.md

## Future Enhancements
- Coverage trend tracking over time
- Integration with code quality tools
- Coverage-based test prioritization
- Automatic coverage regression detection

## Technical Notes
- **Profile Data**: `.profraw` files generated during test execution
- **Merging**: Multiple test runs combined into single `.profdata` file
- **Source Mapping**: Coverage mapped back to original source files
- **Exclusions**: Ability to exclude third-party code and test files
- **Thread Safety**: Profile collection works with multi-threaded tests