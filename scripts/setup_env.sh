#!/usr/bin/env bash

# Spanny3 Development Environment Setup Script
# This script is automatically run when activating the Pixi environment

echo "🚀 Spanny3 Development Environment Activated"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Display tool versions if available
if command -v clang++ &> /dev/null; then
    echo "Clang version: $(clang++ --version | head -n1)"
fi

if command -v cmake &> /dev/null; then
    echo "CMake version: $(cmake --version | head -n1)"
fi

if command -v ninja &> /dev/null; then
    echo "Ninja version: $(ninja --version)"
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Quick commands:"
echo "  pixi run dev         - Configure, build, and test"
echo "  pixi run build       - Build the project"
echo "  pixi run test        - Run tests"
echo "  pixi run coverage    - Generate coverage report"
echo "  pixi run lint        - Run linters and formatters"
echo "  pixi task list       - Show all available tasks"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"