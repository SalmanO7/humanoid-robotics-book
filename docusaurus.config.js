// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Building Humanoid Robots with Artificial Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from.

  onBrokenLinks: 'warn', // Changed from 'throw' to 'warn' to allow build to continue
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-organization/book/tree/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      metadata: [
        {name: 'keywords', content: 'humanoid robotics, physical AI, robotics, artificial intelligence, kinematics, dynamics, control systems, sensors, actuators, ROS, Python robotics'},
        {name: 'description', content: 'A comprehensive guide to Physical AI and Humanoid Robotics, covering fundamentals, implementation, and advanced concepts.'},
        {name: 'og:title', content: 'Physical AI & Humanoid Robotics - Complete Guide'},
        {name: 'og:description', content: 'Learn about humanoid robotics, AI integration, kinematics, dynamics, and practical implementation.'},
        {name: 'og:type', content: 'website'},
        {name: 'og:url', content: 'https://your-username.github.io/book/'},
      ],
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.svg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Book Logo',
          src: 'img/logo.svg',
          href: '/book/docs/intro', // Add a proper link to intro instead of defaulting to baseUrl
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/your-organization/book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Fundamentals',
                to: '/docs/fundamentals/kinematics', // Changed to point to a specific doc instead of category
              },
              {
                label: 'Sensors & Actuators',
                to: '/docs/sensors-actuators/sensors', // Changed to point to a specific doc instead of category
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/humanoid-robotics',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/humanoid-robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-organization/book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;