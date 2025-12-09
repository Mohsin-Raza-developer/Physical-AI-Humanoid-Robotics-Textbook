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
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'ROS 2',
      items: [
        'ros2/intro',
        'ros2/architecture',
        'ros2/nodes-packages',
        'ros2/communication',
        'ros2/launch',
        'ros2/ros2-physical-ai'
      ],
      link: {
        type: 'doc',
        id: 'ros2/intro',
      },
    },
    {
      type: 'category',
      label: 'Gazebo/Unity Simulation',
      items: [
        'gazebo-unity/intro',
        'gazebo-unity/gazebo-fundamentals',
        'gazebo-unity/unity-robotics',
        'gazebo-unity/physics-sensors',
        'gazebo-unity/simulation-physical-ai'
      ],
      link: {
        type: 'doc',
        id: 'gazebo-unity/intro',
      },
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: [
        'isaac/intro',
        'isaac/isaac-sim',
        'isaac/perception-pipelines',
        'isaac/nav-manipulation',
        'isaac/isaac-physical-ai'
      ],
      link: {
        type: 'doc',
        id: 'isaac/intro',
      },
    },
    {
      type: 'category',
      label: 'Vision-Language-Action Models',
      items: [
        'vla/intro',
        'vla/vla-architectures',
        'vla/training-vla',
        'vla/vla-control',
        'vla/vla-physical-ai'
      ],
      link: {
        type: 'doc',
        id: 'vla/intro',
      },
    },
  ],
};

export default sidebars;
