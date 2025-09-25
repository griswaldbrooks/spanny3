[![ci](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml) [![coverage](https://img.shields.io/badge/coverage-report-blue.svg)](https://github.com/griswaldbrooks/spanny3/actions)
# spanny3
Robot arm project for CppCon 2024 presentation.

# development container
Build a new development image
```shell
mkdir -p ~/.spanny3
export USER_UID=$(id -u) && export USER_GID=$(id -g) && docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
cmake -S src/spanny3/ -B artifacts/build
cmake --build artifacts/build
```

# run
```shell
./artifacts/build/rrt_cli
```

# test
```shell
ctest --test-dir artifacts/build
```

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
