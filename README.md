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

# remove orphaned containers
```shell
docker compose -f compose.dev.yml down --remove-orphans
```

# start lsp
```shell
export PORT=5051
socat TCP-LISTEN:${PORT},fork,reuseaddr EXEC:"clangd -log=verbose --background-index --path-mappings='/host/path/to/source=/container/path/to/source'"

```
