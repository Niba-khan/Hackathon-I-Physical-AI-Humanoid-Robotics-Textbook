import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '6fd'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '004'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f89'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'a15'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', '2ae'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ai-robot-brain/chapter-1',
                component: ComponentCreator('/docs/content/modules/ai-robot-brain/chapter-1', '327'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ai-robot-brain/chapter-2',
                component: ComponentCreator('/docs/content/modules/ai-robot-brain/chapter-2', '53a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ai-robot-brain/chapter-3',
                component: ComponentCreator('/docs/content/modules/ai-robot-brain/chapter-3', 'afd'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ai-robot-brain/checklists/validation-checklist',
                component: ComponentCreator('/docs/content/modules/ai-robot-brain/checklists/validation-checklist', 'a07'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ai-robot-brain/intro',
                component: ComponentCreator('/docs/content/modules/ai-robot-brain/intro', 'de6'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/digital-twin/chapter-1',
                component: ComponentCreator('/docs/content/modules/digital-twin/chapter-1', 'e8f'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/digital-twin/chapter-2',
                component: ComponentCreator('/docs/content/modules/digital-twin/chapter-2', '06a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/digital-twin/chapter-3',
                component: ComponentCreator('/docs/content/modules/digital-twin/chapter-3', '675'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/digital-twin/checklists/validation-checklist',
                component: ComponentCreator('/docs/content/modules/digital-twin/checklists/validation-checklist', '825'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/digital-twin/intro',
                component: ComponentCreator('/docs/content/modules/digital-twin/intro', '10e'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ros2-fundamentals/chapter-1',
                component: ComponentCreator('/docs/content/modules/ros2-fundamentals/chapter-1', '07e'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ros2-fundamentals/chapter-2',
                component: ComponentCreator('/docs/content/modules/ros2-fundamentals/chapter-2', 'd87'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ros2-fundamentals/chapter-3',
                component: ComponentCreator('/docs/content/modules/ros2-fundamentals/chapter-3', '160'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ros2-fundamentals/checklists/validation-checklist',
                component: ComponentCreator('/docs/content/modules/ros2-fundamentals/checklists/validation-checklist', 'cf9'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/ros2-fundamentals/intro',
                component: ComponentCreator('/docs/content/modules/ros2-fundamentals/intro', '782'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/vla/chapter-1',
                component: ComponentCreator('/docs/content/modules/vla/chapter-1', '898'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/vla/chapter-2',
                component: ComponentCreator('/docs/content/modules/vla/chapter-2', '253'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/vla/chapter-3',
                component: ComponentCreator('/docs/content/modules/vla/chapter-3', '772'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/vla/checklists/validation-checklist',
                component: ComponentCreator('/docs/content/modules/vla/checklists/validation-checklist', 'e64'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/docs/content/modules/vla/intro',
                component: ComponentCreator('/docs/content/modules/vla/intro', '9c5'),
                exact: true,
                sidebar: "textbookSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '6ce'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
