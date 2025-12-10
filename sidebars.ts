import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main textbook sidebar
  textbookSidebar: [
    // Introduction
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: ['intro/index'],
    },

    // Module 1: ROS 2 (Weeks 3-5)
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-1-ros2/intro',
      },
      items: [
        'module-1-ros2/intro',
        {
          type: 'category',
          label: 'Week 3: ROS 2 Fundamentals',
          items: [
            'module-1-ros2/week-3-lesson-1-ros2-architecture',
            'module-1-ros2/week-3-lesson-2-nodes-packages',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: Advanced Communication',
          items: [
            'module-1-ros2/week-4-lesson-1-services-actions',
          ],
        },
        {
          type: 'category',
          label: 'Week 5: URDF & Launch',
          items: [
            'module-1-ros2/week-5-lesson-1-launch-files',
            'module-1-ros2/week-5-lesson-2-urdf-humanoids',
          ],
        },
      ],
    },

    // Module 2: Gazebo & Unity (Weeks 6-7)
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity (Weeks 6-7)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-2-gazebo-unity/intro',
      },
      items: [
        'module-2-gazebo-unity/intro',
        {
          type: 'category',
          label: 'Week 6: Gazebo Fundamentals',
          items: [
            'module-2-gazebo-unity/week-6-lesson-1-gazebo-setup',
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Physics & Unity',
          items: [
            'module-2-gazebo-unity/week-7-lesson-1-physics-simulation',
            'module-2-gazebo-unity/week-7-lesson-2-unity-robotics',
            'module-2-gazebo-unity/week-7-lesson-3-sensor-simulation',
          ],
        },
      ],
    },

    // Module 3: NVIDIA Isaac (Weeks 8-10)
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-3-isaac/intro',
      },
      items: [
        'module-3-isaac/intro',
        {
          type: 'category',
          label: 'Week 8: Isaac Platform',
          items: [
            'module-3-isaac/week-8-lesson-1-isaac-sim-setup',
          ],
        },
        {
          type: 'category',
          label: 'Week 9: Perception & Manipulation',
          items: [
            'module-3-isaac/week-9-lesson-1-ai-perception',
            'module-3-isaac/week-9-lesson-2-manipulation',
          ],
        },
        {
          type: 'category',
          label: 'Week 10: Sim-to-Real',
          items: [
            'module-3-isaac/week-10-lesson-2-sim-to-real',
          ],
        },
      ],
    },

    // Module 4: VLA & Capstone (Weeks 11-13)
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone (Weeks 11-13)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-4-vla/intro',
      },
      items: [
        'module-4-vla/intro',
        {
          type: 'category',
          label: 'Week 11: Humanoid Robotics',
          items: [
            'module-4-vla/week-11-lesson-1-humanoid-kinematics',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Advanced Interaction',
          items: [
            'module-4-vla/week-12-lesson-1-manipulation-grasping',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Conversational AI & Capstone',
          items: [
            'module-4-vla/week-13-lesson-1-conversational-robotics',
            'module-4-vla/week-13-lesson-3-capstone-project',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
