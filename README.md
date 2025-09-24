[![ci](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny3/actions/workflows/ci.yml) [![coverage](https://img.shields.io/badge/coverage-report-blue.svg)](https://github.com/griswaldbrooks/spanny3/actions)
# spanny3
Robot arm project for CppCon 2024 presentation.

# development container
Build a new development image
```shell
mkdir -p ~/.spanny3/ccache
export UID=$(id -u) export GID=$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
cmake -S src/spanny3/ -B build
cmake --build build
```

# run
```shell
./build/rrt_cli
```

# test
```shell
ctest --test-dir build
```

# coverage
Generate and view code coverage reports:
```shell
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm development bash -c "cmake -S src/spanny3/ -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build && cmake --build build --target coverage"
```
View HTML coverage report by opening `build/coverage/reports/html/index.html` in your browser (from within the container), or copy reports to host:
```shell
export GID=$(id -g) && docker compose -f compose.dev.yml run --rm -v $(pwd)/coverage-reports:/tmp/reports:rw development bash -c "cmake -S src/spanny3/ -B build -DCMAKE_BUILD_TYPE=Coverage && cmake --build build && cmake --build build --target coverage && sudo cp -r build/coverage/reports/* /tmp/reports/"
```
Then open `coverage-reports/html/index.html` in your browser.

# remove orphaned containers
```shell
docker compose -f compose.dev.yml down --remove-orphans
```

# start lsp
```shell
export PORT=5051
socat TCP-LISTEN:${PORT},fork,reuseaddr EXEC:"clangd -log=verbose --background-index --path-mappings='/host/path/to/source=/container/path/to/source'"

```
