# Spanny3 Documentation

This directory contains the Docusaurus-based documentation website for Spanny3.

## Documentation Structure

```
docs/
├── docs/                       # Markdown documentation
│   ├── getting-started/       # Installation and setup guides
│   ├── algorithm/             # RRT algorithm documentation
│   ├── development/           # Development workflows
│   ├── examples/              # Code examples
│   └── api/                   # Manual API documentation (user-friendly)
├── static/
│   └── doxygen/               # Auto-generated Doxygen API reference
├── src/                       # React components
├── docusaurus.config.ts       # Docusaurus configuration
└── sidebars.ts                # Sidebar navigation configuration
```

## Local Development

### Prerequisites

- Node.js 20 or higher
- npm

### Install Dependencies

```bash
cd docs
npm install
```

### Start Development Server

```bash
npm start
```

This opens `http://localhost:3000` with live reload.

### Build for Production

```bash
npm run build
```

Output goes to `build/` directory.

### Preview Production Build

```bash
npm run serve
```

## Documentation Types

### 1. User Documentation (Docusaurus)

**Location**: `docs/docs/`

Manual documentation written in Markdown:
- Getting Started guides
- Algorithm explanations
- Development workflows
- Code examples

**Pros**: User-friendly, searchable, consistent styling

### 2. API Reference (Doxygen)

**Location**: Generated to `static/doxygen/`

Auto-generated from C++ source code comments:
- Complete API reference
- Class diagrams
- Call graphs
- Source browsing

**Pros**: Always up-to-date, comprehensive, auto-generated

### Accessing Both

- **User Docs**: https://griswaldbrooks.github.io/spanny3/
- **API Reference**: https://griswaldbrooks.github.io/spanny3/doxygen/html/index.html

## Deployment

### GitHub Pages (Automated)

Documentation automatically deploys on push to `main`:

1. GitHub Actions runs `.github/workflows/deploy-docs.yml`
2. Generates Doxygen documentation
3. Builds Docusaurus website
4. Deploys to `gh-pages` branch
5. GitHub Pages serves from `gh-pages`

**URL**: https://griswaldbrooks.github.io/spanny3/

### Manual Deployment

```bash
# Set Git user for deployment
GIT_USER=<your-github-username> npm run deploy
```

## Writing Documentation

### Adding a New Page

1. Create Markdown file in appropriate directory:
   ```bash
   touch docs/examples/my-new-example.md
   ```

2. Add frontmatter:
   ```markdown
   ---
   sidebar_position: 1
   ---

   # My New Example

   Content here...
   ```

3. Update `sidebars.ts` if needed:
   ```ts
   items: [
     'examples/my-new-example',
   ],
   ```

### Code Examples

Use language-specific syntax highlighting:

````markdown
```cpp
auto node = spanny::node_t{spanny::position_t{0.0, 0.0}};
```
````

Supported languages: cpp, cmake, bash, json, typescript, jsx

### Links

- **Internal docs**: `[Link Text](../path/to/doc.md)`
- **External**: `[GitHub](https://github.com/griswaldbrooks/spanny3)`
- **Doxygen**: `[API Ref](/doxygen/html/index.html)`

## Configuration

### docusaurus.config.ts

Main configuration:
- Site metadata
- Theme settings
- Navbar/footer
- Plugins
- Base URL: `/spanny3/` for GitHub Pages

### sidebars.ts

Defines documentation structure:
- `docsSidebar`: User documentation
- `apiSidebar`: Manual API guide

## Updating Doxygen

1. Update C++ source comments
2. Regenerate locally (optional):
   ```bash
   doxygen Doxyfile
   ```
3. Push to `main`
4. CI regenerates and deploys automatically

## Troubleshooting

### Port Already in Use

```bash
npm start -- --port 3001
```

### Build Errors

Clear cache and rebuild:
```bash
npm run clear
npm run build
```

### Doxygen Not Found

Install on your system:
```bash
# macOS
brew install doxygen graphviz

# Ubuntu/Debian
sudo apt-get install doxygen graphviz
```

## Links

- **Live Docs**: https://griswaldbrooks.github.io/spanny3/
- **Docusaurus Docs**: https://docusaurus.io/docs
- **Doxygen Manual**: https://www.doxygen.nl/manual/
