// @ts-check

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'your-project',

  onBrokenLinks: 'throw',
  plugins: [
  require.resolve('./src/plugins/chatbotPlugin.js'),
],

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          routeBasePath: '/',   //  ⛔ DEFAULT PAGE REMOVED — DOCS = HOMEPAGE
          sidebarPath: './sidebars.js',
          editUrl: null,
        },
        blog: false,             //  ⛔ BLOG REMOVED (like your friend's)
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig: ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },

      navbar: {
        title: 'Humanoid Robotics',
        logo: {
          alt: 'Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },

     footer: {
  style: 'dark',
  links: [
    {
      title: 'Book',
      items: [
        {
          label: 'Book',
          to: '/',     
        },
      ],
    },
    {
      title: 'GitHub',
      items: [
        {
          label: 'GitHub',
          href: 'https://github.com/',
        },
      ],
    },
  ],
  copyright:
    `Copyright © ${new Date().getFullYear()} Physical AI`,
},


      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
