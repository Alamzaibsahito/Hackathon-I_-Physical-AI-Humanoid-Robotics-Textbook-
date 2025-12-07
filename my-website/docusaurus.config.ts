import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence for the Real World',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'facebook',
  projectName: 'docusaurus',

  onBrokenLinks: 'throw',

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
          routeBasePath: '/',
          editUrl: 'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI TextBook',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
        href: '/', // Link logo to homepage that handles auth
      },
      items: [
        {
          to: '/',
          label: 'Home',
          position: 'left',
        },
        {
          to: '/preface/intro',
          label: 'Textbook',
          position: 'left',
        },
        {
          href: 'https://github.com/Alamzaibsahito?tab=repositories',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Physical AI TextBook',
          items: [
            {
              label: 'Introduction',
              to: '/preface/intro',
            },
            {
              label: 'Chapters',
              to: '/preface/intro',
            },
            {
              label: 'Contribute',
              to: '/preface/contributing',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Alamzaibsahito?tab=repositories',
            },
            {
              label: 'Community',
              href: 'https://github.com/Alamzaibsahito?tab=repositories',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Documentation',
              to: '/preface/intro',
            },
            {
              label: 'Glossary',
              to: '/glossary',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
