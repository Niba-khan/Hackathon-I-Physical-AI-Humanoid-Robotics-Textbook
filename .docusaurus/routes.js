import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon-textbook/markdown-page',
    component: ComponentCreator('/hackathon-textbook/markdown-page', '478'),
    exact: true
  },
  {
    path: '/hackathon-textbook/docs',
    component: ComponentCreator('/hackathon-textbook/docs', 'e0e'),
    routes: [
      {
        path: '/hackathon-textbook/docs',
        component: ComponentCreator('/hackathon-textbook/docs', 'cfd'),
        routes: [
          {
            path: '/hackathon-textbook/docs',
            component: ComponentCreator('/hackathon-textbook/docs', '55f'),
            routes: [
              {
                path: '/hackathon-textbook/docs/',
                component: ComponentCreator('/hackathon-textbook/docs/', '845'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-1',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-1', 'bcf'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-2',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-2', 'cb0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-3',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ai-robot-brain/chapter-3', '79b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ai-robot-brain/checklists/validation-checklist',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ai-robot-brain/checklists/validation-checklist', '377'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ai-robot-brain/intro',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ai-robot-brain/intro', '0de'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/digital-twin/chapter-1',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/digital-twin/chapter-1', '15f'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/digital-twin/chapter-2',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/digital-twin/chapter-2', '1f1'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/digital-twin/chapter-3',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/digital-twin/chapter-3', 'f6c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/digital-twin/checklists/validation-checklist',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/digital-twin/checklists/validation-checklist', '181'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/digital-twin/intro',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/digital-twin/intro', 'e31'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-1',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-1', '211'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-2',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-2', 'f60'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-3',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ros2-fundamentals/chapter-3', '6a7'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ros2-fundamentals/checklists/validation-checklist',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ros2-fundamentals/checklists/validation-checklist', 'c94'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/ros2-fundamentals/intro',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/ros2-fundamentals/intro', '742'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/vla/chapter-1',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/vla/chapter-1', '861'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/vla/chapter-2',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/vla/chapter-2', 'b99'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/vla/chapter-3',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/vla/chapter-3', '149'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/vla/checklists/validation-checklist',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/vla/checklists/validation-checklist', 'b64'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/hackathon-textbook/docs/content/modules/vla/intro',
                component: ComponentCreator('/hackathon-textbook/docs/content/modules/vla/intro', '694'),
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
    path: '/hackathon-textbook/',
    component: ComponentCreator('/hackathon-textbook/', '980'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
