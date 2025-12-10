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
    // Introduction (Weeks 1-2)
    {
      type: 'category',
      label: 'Introduction (Weeks 1-2)',
      collapsed: false,
      link: {
        type: 'doc',
        id: 'intro/index',
      },
      items: [
        'intro/index',
        'intro/week-1-2-physical-ai-foundations',
        'intro/week-1-2-sensors-overview',
      ],
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
            'module-1-ros2/week-4-lesson-2-building-packages',
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
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-1-ros2/assessments/ros2-package-project',
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
            'module-2-gazebo-unity/week-6-lesson-2-urdf-sdf',
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
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-2-gazebo-unity/assessments/gazebo-simulation-project',
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
            'module-3-isaac/week-8-lesson-2-isaac-sdk-intro',
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
            'module-3-isaac/week-10-lesson-1-reinforcement-learning',
            'module-3-isaac/week-10-lesson-2-sim-to-real',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-3-isaac/assessments/isaac-perception-pipeline',
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
            'module-4-vla/week-11-lesson-2-bipedal-locomotion',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Advanced Interaction',
          items: [
            'module-4-vla/week-12-lesson-1-manipulation-grasping',
            'module-4-vla/week-12-lesson-2-human-robot-interaction',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Conversational AI & Capstone',
          items: [
            'module-4-vla/week-13-lesson-1-conversational-robotics',
            'module-4-vla/week-13-lesson-2-gpt-integration',
            'module-4-vla/week-13-lesson-3-capstone-project',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-4-vla/assessments/capstone-simulated-humanoid',
          ],
        },
      ],
    },

    // Setup & Resources
    {
      type: 'category',
      label: 'Setup & Resources',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Hardware & Software Setup',
          items: [
            'setup/hardware-requirements',
            'setup/software-setup',
            'setup/lab-infrastructure',
            'setup/student-kit-guide',
          ],
        },
        {
          type: 'category',
          label: 'Resources',
          items: [
            'resources/glossary',
            'resources/references',
            'resources/additional-reading',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
