// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        {
          type: 'doc',
          id: 'getting-started/overview',
          label: 'Overview',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Fundamentals',
      items: [
        {
          type: 'doc',
          id: 'fundamentals/kinematics',
          label: 'Kinematics',
        },
        {
          type: 'doc',
          id: 'fundamentals/dynamics',
          label: 'Dynamics',
        },
        {
          type: 'doc',
          id: 'fundamentals/control-systems',
          label: 'Control Systems',
        },
        {
          type: 'doc',
          id: 'fundamentals/locomotion',
          label: 'Locomotion',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Sensors & Actuators',
      items: [
        {
          type: 'doc',
          id: 'sensors-actuators/sensors',
          label: 'Sensors',
        },
        {
          type: 'doc',
          id: 'sensors-actuators/actuators',
          label: 'Actuators',
        },
        {
          type: 'doc',
          id: 'sensors-actuators/integration',
          label: 'Integration',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'AI in Humanoid Robotics',
      items: [
        {
          type: 'doc',
          id: 'ai-humanoid/perception',
          label: 'Perception',
        },
        {
          type: 'doc',
          id: 'ai-humanoid/planning',
          label: 'Planning',
        },
        {
          type: 'doc',
          id: 'ai-humanoid/learning',
          label: 'Learning',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Code Examples',
      items: [
        {
          type: 'doc',
          id: 'code-examples/python-basics',
          label: 'Python Basics',
        },
        {
          type: 'doc',
          id: 'code-examples/ros-integration',
          label: 'ROS Integration',
        },
        {
          type: 'doc',
          id: 'code-examples/simulation',
          label: 'Simulation',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Interactive Demos',
      items: [
        {
          type: 'doc',
          id: 'interactive-demos/mcp-integration',
          label: 'MCP Integration',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        {
          type: 'doc',
          id: 'reference/glossary',
          label: 'Glossary',
        },
        {
          type: 'doc',
          id: 'reference/bibliography',
          label: 'Bibliography',
        },
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;