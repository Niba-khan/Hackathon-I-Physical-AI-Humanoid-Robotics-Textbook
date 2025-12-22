
// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'PHYSICAL AI & HUMANOID ROBOTICS – AI-NATIVE TEXTBOOK',
  tagline:
    'ROS 2 • Gazebo • Isaac • Digital Twin • Vision-Language-Action Robotics',
  favicon: 'img/favicon.ico',

  // ✅ VERCEL CONFIG
  url: 'https://hackathon-i-physical-ai-humanoid-ro-six.vercel.app',
  baseUrl: '/',

  organizationName: 'Niba-khan',
  projectName: 'Hackathon-I-Physical-AI-Humanoid-Robotics-Textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/Niba-khan/Hackathon-I-Physical-AI-Humanoid-Robotics-Textbook/tree/main/',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/Niba-khan/Hackathon-I-Physical-AI-Humanoid-Robotics-Textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2',
              to: '/docs/content/modules/ros2-fundamentals/intro',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/content/modules/digital-twin/intro',
            },
            {
              label: 'Module 3: AI-Robot Brain',
              to: '/docs/content/modules/ai-robot-brain/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action',
              to: '/docs/content/modules/vla/intro',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Niba-khan/Hackathon-I-Physical-AI-Humanoid-Robotics-Textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics – AI-Native Textbook.`,
    },

    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  },
};

module.exports = config;
