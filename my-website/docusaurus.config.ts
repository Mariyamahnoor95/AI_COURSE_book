import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An interactive textbook for embodied AI, robotics simulation, and humanoid systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Mariyamahnoor95.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/AI_COURSE_book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Mariyamahnoor95', // Usually your GitHub org/user name.
  projectName: 'AI_COURSE_book', // Usually your repo name.

  onBrokenLinks: 'warn', // Allow build with broken links during development

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',

    // Enhanced metadata for SEO
    metadata: [
      {name: 'keywords', content: 'robotics, AI, humanoid robotics, ROS 2, physical AI, embodied intelligence, Isaac Sim, VLA models'},
      {name: 'description', content: 'Interactive textbook for learning physical AI, humanoid robotics, ROS 2, and vision-language-action systems'},
    ],

    // Color mode configuration
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    // Enhanced navbar
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      hideOnScroll: false,
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo.svg',
        width: 32,
        height: 32,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: '📚 Textbook',
        },
        {
          type: 'dropdown',
          label: '🎓 Modules',
          position: 'left',
          items: [
            {
              label: 'Foundations',
              to: '/docs/foundations',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-01-ros2',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/module-02-digital-twin',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-03-isaac',
            },
            {
              label: 'Module 4: VLA Integration',
              to: '/docs/module-04-vla',
            },
          ],
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/noori/hackathon1_book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    // Enhanced footer
    footer: {
      style: 'dark',
      links: [
        {
          title: '📖 Textbook Modules',
          items: [
            {
              label: '🔧 Foundations',
              to: '/docs/foundations',
            },
            {
              label: '🤖 ROS 2',
              to: '/docs/module-01-ros2',
            },
            {
              label: '🔄 Digital Twin',
              to: '/docs/module-02-digital-twin',
            },
            {
              label: '⚡ NVIDIA Isaac',
              to: '/docs/module-03-isaac',
            },
            {
              label: '🧠 VLA Integration',
              to: '/docs/module-04-vla',
            },
          ],
        },
        {
          title: '🛠️ Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/noori/hackathon1_book',
            },
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
          ],
        },
        {
          title: '🌐 Community',
          items: [
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
            {
              label: 'Robotics Stack Exchange',
              href: 'https://robotics.stackexchange.com/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },

    // Enhanced Prism theme with additional languages
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'yaml', 'cmake', 'json'],
      magicComments: [
        {
          className: 'theme-code-block-highlighted-line',
          line: 'highlight-next-line',
          block: {start: 'highlight-start', end: 'highlight-end'},
        },
        {
          className: 'code-block-error-line',
          line: 'This will error',
        },
      ],
    },

    // Table of contents settings
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },

    // Docs sidebar
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
