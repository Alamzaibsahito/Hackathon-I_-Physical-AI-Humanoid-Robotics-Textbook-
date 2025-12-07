import React from 'react';
import { useAuth } from '../context/AuthContext';
import { motion, AnimatePresence } from 'framer-motion';
import RobotAnimation from './RobotAnimation';

const BookContent = () => {
  const { user } = useAuth();

  return (
    <AnimatePresence mode="wait">
      <motion.div
        key="book-content"
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        exit={{ opacity: 0 }}
        transition={{ duration: 0.5 }}
        className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-900 dark:to-gray-800 py-12"
      >
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <motion.div
            initial={{ opacity: 0, y: -20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 0.1 }}
            className="text-center mb-12"
          >
            <h1 className="text-4xl md:text-5xl font-bold text-gray-900 dark:text-white mb-4">
              Physical AI & Humanoid Robotics Textbook
            </h1>
            <p className="text-xl text-gray-600 dark:text-gray-300 max-w-3xl mx-auto">
              Welcome {user?.name || 'Learner'}! Your personalized learning journey begins here.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8">
            {/* Preface Section */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.2 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-blue-500 text-2xl mr-3">📚</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Preface</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/preface/intro" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Introduction
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/preface/how-to-use" className="text-blue-600 dark:text-blue-400 hover:underline">
                    How to Use This Book
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/preface/conventions" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Conventions
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/preface/contributing" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Contributing
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/preface/feedback" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Feedback
                  </a>
                </motion.li>
              </ul>
            </motion.div>

            {/* Chapter 1: Introduction to Physical AI */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.3 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-purple-500 text-2xl mr-3">🤖</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Chapter 1: Introduction to Physical AI</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/introduction-physical-ai/what-is-physical-ai" className="text-blue-600 dark:text-blue-400 hover:underline">
                    What is Physical AI?
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/introduction-physical-ai/embodied-intelligence" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Embodied Intelligence
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/introduction-physical-ai/humanoid-future" className="text-blue-600 dark:text-blue-400 hover:underline">
                    The Humanoid Future
                  </a>
                </motion.li>
              </ul>
            </motion.div>

            {/* Chapter 2: ROS 2 Basics */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.4 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-green-500 text-2xl mr-3">🌐</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Chapter 2: ROS 2 Basics</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/ros2-basics/ros2-overview" className="text-blue-600 dark:text-blue-400 hover:underline">
                    ROS 2 Overview
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/ros2-basics/nodes-topics-services" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Nodes, Topics, and Services
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/ros2-basics/rclpy-basics" className="text-blue-600 dark:text-blue-400 hover:underline">
                    rclpy Basics
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/ros2-basics/urdf-for-humanoids" className="text-blue-600 dark:text-blue-400 hover:underline">
                    URDF for Humanoids
                  </a>
                </motion.li>
              </ul>
            </motion.div>

            {/* Chapter 3: Digital Twin Simulation */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.5 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-yellow-500 text-2xl mr-3">🎮</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Chapter 3: Digital Twin Simulation</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/digital-twin-simulation/gazebo-basics" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Gazebo Basics
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/digital-twin-simulation/unity-visualization" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Unity Visualization
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/digital-twin-simulation/physics-simulation" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Physics Simulation
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/digital-twin-simulation/sensors-lidar-depth" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Sensors: LiDAR and Depth
                  </a>
                </motion.li>
              </ul>
            </motion.div>

            {/* Chapter 4: NVIDIA Isaac Platform */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.6 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-red-500 text-2xl mr-3">🚀</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Chapter 4: NVIDIA Isaac Platform</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/nvidia-isaac-platform/isaac-sim-overview" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Isaac Sim Overview
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/nvidia-isaac-platform/isaac-ros" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Isaac ROS
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/nvidia-isaac-platform/vslam-and-navigation" className="text-blue-600 dark:text-blue-400 hover:underline">
                    VSLAM and Navigation
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/nvidia-isaac-platform/nav2-path-planning" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Nav2 Path Planning
                  </a>
                </motion.li>
              </ul>
            </motion.div>

            {/* Chapter 5: Vision-Language-Action */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.7 }}
              whileHover={{ y: -5 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6 hover:shadow-xl transition-all duration-300"
            >
              <div className="flex items-center mb-4">
                <div className="text-pink-500 text-2xl mr-3">👁️</div>
                <h3 className="text-xl font-bold text-gray-800 dark:text-white">Chapter 5: Vision-Language-Action</h3>
              </div>
              <ul className="space-y-2">
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/vision-language-action/vla-concepts" className="text-blue-600 dark:text-blue-400 hover:underline">
                    VLA Concepts
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/vision-language-action/llm-planning" className="text-blue-600 dark:text-blue-400 hover:underline">
                    LLM Planning
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/vision-language-action/whisper-voice-to-action" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Whisper: Voice to Action
                  </a>
                </motion.li>
                <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                  <a href="/vision-language-action/multi-step-robot-task" className="text-blue-600 dark:text-blue-400 hover:underline">
                    Multi-Step Robot Tasks
                  </a>
                </motion.li>
              </ul>
            </motion.div>
          </div>

          {/* Additional chapters section */}
          <div className="mt-16 grid grid-cols-1 md:grid-cols-2 gap-8">
            {/* Chapter 6-8 */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.8 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6"
            >
              <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">Chapters 6-8: Advanced Topics</h3>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div>
                  <h4 className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Chapter 6</h4>
                  <ul className="space-y-1 text-sm">
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/humanoid-development/kinematics-dynamics" className="text-blue-600 dark:text-blue-400 hover:underline">Kinematics & Dynamics</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/humanoid-development/bipedal-walking" className="text-blue-600 dark:text-blue-400 hover:underline">Bipedal Walking</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/humanoid-development/manipulation-grasping" className="text-blue-600 dark:text-blue-400 hover:underline">Manipulation & Grasping</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/humanoid-development/hri-design" className="text-blue-600 dark:text-blue-400 hover:underline">HRI Design</a>
                    </motion.li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Chapter 7</h4>
                  <ul className="space-y-1 text-sm">
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/conversational-robotics/speech-recognition" className="text-blue-600 dark:text-blue-400 hover:underline">Speech Recognition</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/conversational-robotics/gpt-integration" className="text-blue-600 dark:text-blue-400 hover:underline">GPT Integration</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/conversational-robotics/multimodal-interaction" className="text-blue-600 dark:text-blue-400 hover:underline">Multimodal Interaction</a>
                    </motion.li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Chapter 8</h4>
                  <ul className="space-y-1 text-sm">
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/hardware-requirements/digital-twin-workstation" className="text-blue-600 dark:text-blue-400 hover:underline">Digital Twin Workstation</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/hardware-requirements/jetson-edge-kit" className="text-blue-600 dark:text-blue-400 hover:underline">Jetson Edge Kit</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/hardware-requirements/robot-lab-options" className="text-blue-600 dark:text-blue-400 hover:underline">Robot Lab Options</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/hardware-requirements/cloud-infrastructure" className="text-blue-600 dark:text-blue-400 hover:underline">Cloud Infrastructure</a>
                    </motion.li>
                  </ul>
                </div>
              </div>
            </motion.div>

            {/* Chapter 9-10 */}
            <motion.div
              initial={{ opacity: 0, y: 20, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.9 }}
              className="bg-white dark:bg-gray-800 rounded-xl shadow-lg p-6"
            >
              <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">Chapters 9-10: Assessment & Reference</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <h4 className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Chapter 9: Assessments</h4>
                  <ul className="space-y-1 text-sm">
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/assessments/ros2-project" className="text-blue-600 dark:text-blue-400 hover:underline">ROS 2 Project</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/assessments/isaac-perception" className="text-blue-600 dark:text-blue-400 hover:underline">Isaac Perception</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/assessments/gazebo-sim" className="text-blue-600 dark:text-blue-400 hover:underline">Gazebo Sim</a>
                    </motion.li>
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/assessments/capstone-humanoid" className="text-blue-600 dark:text-blue-400 hover:underline">Capstone: Humanoid</a>
                    </motion.li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Chapter 10: Reference</h4>
                  <ul className="space-y-1 text-sm">
                    <motion.li whileHover={{ x: 5 }} transition={{ type: "spring", stiffness: 300 }}>
                      <a href="/glossary" className="text-blue-600 dark:text-blue-400 hover:underline">Glossary</a>
                    </motion.li>
                  </ul>
                </div>
              </div>
            </motion.div>
          </div>

          {/* Call to action */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 1.0 }}
            className="mt-16 text-center"
          >
            <motion.div
              whileHover={{ scale: 1.02 }}
              transition={{ type: "spring", stiffness: 300 }}
              className="bg-gradient-to-r from-blue-500 to-purple-600 rounded-2xl p-1 inline-block"
            >
              <div className="bg-white dark:bg-gray-800 rounded-xl p-8 max-w-2xl">
                <RobotAnimation size="md" animationType="wave" className="mx-auto mb-4" />
                <h3 className="text-2xl font-bold text-gray-800 dark:text-white mb-3">Ready to Dive In?</h3>
                <p className="text-gray-600 dark:text-gray-300 mb-6">
                  Start with the Introduction chapter to get familiar with Physical AI concepts and methodologies.
                </p>
                <motion.a
                  href="/preface/intro"
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                  className="inline-block bg-gradient-to-r from-blue-500 to-purple-600 text-white px-6 py-3 rounded-lg font-medium hover:shadow-lg transition-shadow duration-300"
                >
                  Begin Your Journey
                </motion.a>
              </div>
            </motion.div>
          </motion.div>
        </div>
      </motion.div>
    </AnimatePresence>
  );
};

export default BookContent;