import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'getting-started/installation',
        'getting-started/pixi-setup',
        'getting-started/first-run',
      ],
    },
    {
      type: 'category',
      label: 'Algorithm',
      collapsed: false,
      items: [
        'algorithm/rrt-overview',
        'algorithm/collision-detection',
        'algorithm/performance-analysis',
      ],
    },
    {
      type: 'category',
      label: 'Development',
      collapsed: false,
      items: [
        'development/pixi-workflow',
        'development/testing-guide',
        'development/contributing',
      ],
    },
    {
      type: 'category',
      label: 'Examples',
      collapsed: false,
      items: [
        'examples/basic-planning',
        'examples/custom-obstacles',
        'examples/benchmarking',
      ],
    },
  ],
  apiSidebar: [
    {
      type: 'category',
      label: 'API Reference',
      collapsed: false,
      items: [
        'api/core-types',
        'api/planning-context',
        'api/rrt-planner',
        'api/testing-utilities',
      ],
    },
  ],
};

export default sidebars;
