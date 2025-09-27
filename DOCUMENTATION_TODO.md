# Documentation TODO

This file tracks the remaining work for the Docusaurus documentation site.

## Current Status (2025-09-26)

**Branch**: `feature/add-docusaurus-docs`
**Commit**: `1d38199`

### ✅ Completed

1. **Docusaurus Setup**
   - Initialized Docusaurus v3 with TypeScript
   - Configured for GitHub Pages (`/spanny3/` base URL)
   - Added C++, CMake, Bash, JSON syntax highlighting
   - Created sidebars with separate docs and API sections

2. **Doxygen Integration**
   - Created `Doxyfile` configuration
   - Output set to `docs/static/doxygen/html/`
   - Configured to extract from `include/`, `src/`, `benchmark/`

3. **GitHub Actions Workflow**
   - Created `.github/workflows/deploy-docs.yml`
   - Generates Doxygen → Builds Docusaurus → Deploys to `gh-pages`
   - Runs on push to `main`

4. **Initial Documentation Pages**
   - `docs/docs/getting-started/installation.md` - Pixi installation guide
   - `docs/docs/getting-started/pixi-setup.md` - Development workflow
   - `docs/docs/getting-started/first-run.md` - Quick start tutorial
   - `docs/docs/api/core-types.md` - Manual API docs with examples

5. **Pre-commit Fixes**
   - Updated `.pre-commit-config.yaml` to exclude:
     - `docs/tsconfig.json` (TypeScript allows comments)
     - `docs/package-lock.json` (auto-generated, has false positives)

## 🔴 Known Issues

### Pre-commit Lint Failures

**Problem**: CI still failing on lint checks despite exclusions.

**Files with issues**:
- `docs/tsconfig.json` - JSON validation fails (TypeScript uses JSONC)
- `docs/package-lock.json` - Codespell false positives

**Exclusion added**:
```yaml
exclude: '^(third_party/.*|docs/(package-lock\.json|tsconfig\.json))$'
```

**Next steps**:
1. Verify exclusion pattern works correctly
2. Test with `pixi run lint` locally
3. May need to adjust pattern or add more specific exclusions
4. Consider adding `.codespell_words` entries if needed

### GitHub Pages Not Yet Enabled

**Problem**: Repository settings need manual configuration.

**Steps to enable**:
1. Go to repo Settings → Pages
2. Source: Deploy from a branch
3. Branch: `gh-pages` / `/ (root)`
4. Wait for first deployment after merge to `main`

## 📋 Remaining Documentation Pages

Based on sidebar configuration in `docs/sidebars.ts`:

### Algorithm Section
- [ ] `docs/algorithm/rrt-overview.md` - RRT algorithm explanation
- [ ] `docs/algorithm/collision-detection.md` - Line-circle intersection math
- [ ] `docs/algorithm/performance-analysis.md` - Benchmark results and complexity

### Development Section
- [ ] `docs/development/pixi-workflow.md` - Advanced Pixi workflows
- [ ] `docs/development/testing-guide.md` - Writing and running tests
- [ ] `docs/development/contributing.md` - Contribution guidelines

### Examples Section
- [ ] `docs/examples/basic-planning.md` - Simple planning example code
- [ ] `docs/examples/custom-obstacles.md` - Creating complex scenarios
- [ ] `docs/examples/benchmarking.md` - Running and interpreting benchmarks

### API Section
- [ ] `docs/api/planning-context.md` - `planning_context_t` configuration guide
- [ ] `docs/api/rrt-planner.md` - Using `rrt_t` class
- [ ] `docs/api/testing-utilities.md` - Mock utilities for testing

## 🎯 Quick Fixes for Next Agent

### 1. Fix Pre-commit Lint (Priority: HIGH)

Test the exclusion pattern:
```bash
pixi run lint
```

If still failing, try alternative approaches:
- Add individual file exclusions to specific hooks
- Create `.codespell_words` file for false positives
- Consider disabling checks for `docs/` entirely

### 2. Test Local Documentation Build

```bash
cd docs
npm install
npm start  # Should open http://localhost:3000
```

Verify:
- All links work
- No broken references
- Syntax highlighting works for C++ code blocks

### 3. Verify GitHub Actions Workflow

After merge to `main`:
- Check Actions tab for workflow run
- Verify Doxygen generation succeeds
- Check `gh-pages` branch created
- Enable GitHub Pages in settings

## 📚 Documentation Writing Guide

### File Template

```markdown
---
sidebar_position: 1
---

# Page Title

Brief introduction paragraph.

## Section

Content with code examples:

\`\`\`cpp
#include "spanny/rrt.hpp"

auto context = spanny::planning_context_t{...};
\`\`\`

### Subsection

More details.

## See Also

- [Related Page](../path/to/page.md)
- [Doxygen Reference](/doxygen/html/)
```

### Code Block Languages

- `cpp` - C++ code
- `cmake` - CMake
- `bash` - Shell commands
- `json` - JSON config files
- `typescript` - TypeScript (for Docusaurus config)

### Internal Links

- Same directory: `[Link](./page.md)`
- Parent directory: `[Link](../path/page.md)`
- Root: `[Link](/docs/path/page.md)`
- Doxygen: `[API Ref](/doxygen/html/)`

## 🔗 References

- **Docusaurus Docs**: https://docusaurus.io/docs
- **Doxygen Manual**: https://www.doxygen.nl/manual/
- **GitHub Pages**: https://docs.github.com/en/pages
- **Live Site** (after deploy): https://griswaldbrooks.github.io/spanny3/

## 💡 Tips for Next Agent

1. **Start with pre-commit fix** - This is blocking CI
2. **Write one page at a time** - Don't try to complete all at once
3. **Copy structure from existing pages** - Use `core-types.md` as template
4. **Test locally** - Always preview with `npm start` before committing
5. **Link between pages** - Create a web of cross-references
6. **Add code examples** - Real, runnable code is most valuable
7. **Keep it concise** - Users want quick answers, not essays

## 📝 Notes

- Documentation source lives in `main` branch (`docs/` directory)
- Built artifacts go to `gh-pages` branch (auto-managed by CI)
- Doxygen output not committed to `main` (generated by CI)
- Node.js 20+ required for Docusaurus build
- Docusaurus uses hot reload for live preview during development
