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
  // Manual sidebar configuration with correct document IDs
  bookSidebar: [
    {
      type: 'category',
      label: 'Preface',
      items: [
        'preface/intro',
        'preface/how-to-use',
        'preface/conventions',
        'preface/contributing',
        'preface/feedback',
      ],
    },
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      items: [
        'introduction-physical-ai/what-is-physical-ai',
        'introduction-physical-ai/embodied-intelligence',
        'introduction-physical-ai/humanoid-future',
      ],
    },
    {
      type: 'category',
      label: 'ROS 2 Basics',
      items: [
        'ros2-basics/ros2-overview',
        'ros2-basics/nodes-topics-services',
        'ros2-basics/rclpy-basics',
        'ros2-basics/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'digital-twin-simulation/gazebo-basics',
        'digital-twin-simulation/physics-simulation',
        'digital-twin-simulation/sensors-lidar-depth',
        'digital-twin-simulation/unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Platform',
      items: [
        'nvidia-isaac-platform/isaac-sim-overview',
        'nvidia-isaac-platform/isaac-ros',
        'nvidia-isaac-platform/vslam-and-navigation',
        'nvidia-isaac-platform/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Vision Language Action',
      items: [
        'vision-language-action/vla-concepts',
        'vision-language-action/llm-planning',
        'vision-language-action/whisper-voice-to-action',
        'vision-language-action/multi-step-robot-task',
      ],
    },
    {
      type: 'category',
      label: 'Humanoid Development',
      items: [
        'humanoid-development/kinematics-dynamics',
        'humanoid-development/bipedal-walking',
        'humanoid-development/manipulation-grasping',
        'humanoid-development/hri-design',
      ],
    },
    {
      type: 'category',
      label: 'Conversational Robotics',
      items: [
        'conversational-robotics/speech-recognition',
        'conversational-robotics/gpt-integration',
        'conversational-robotics/multimodal-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware-requirements/digital-twin-workstation',
        'hardware-requirements/jetson-edge-kit',
        'hardware-requirements/robot-lab-options',
        'hardware-requirements/cloud-infrastructure',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/ros2-project',
        'assessments/gazebo-sim',
        'assessments/isaac-perception',
        'assessments/capstone-humanoid',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
