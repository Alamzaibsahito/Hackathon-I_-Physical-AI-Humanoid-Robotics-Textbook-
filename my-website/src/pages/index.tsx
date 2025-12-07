import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import { motion } from 'framer-motion';
import RobotAnimation from '../components/RobotAnimation';
import AnimatedLoginButton from '../components/AnimatedLoginButton';
import LoginModal from '../components/LoginModal';
import Navbar from '../components/Navbar';
import BookContent from '../components/BookContent';

const HomePage = () => {
  const { user, loading } = useAuth();
  const [isLoginModalOpen, setIsLoginModalOpen] = useState(false);
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // Show loading state on the server or during initial load
  if (!isClient || loading) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-900 dark:to-gray-800">
        <Navbar />
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12">
          <div className="text-center">
            <div className="mb-8">
              <h1 className="text-4xl md:text-6xl font-extrabold text-gray-900 dark:text-white mb-4">
                Physical AI & Humanoid Robotics
              </h1>
              <p className="text-xl text-gray-600 dark:text-gray-300 max-w-3xl mx-auto">
                Embodied Intelligence for the Real World - Learn cutting-edge concepts in AI and robotics
              </p>
            </div>

            <div className="mb-12">
              <div className="w-24 h-24 mx-auto bg-gray-200 dark:bg-gray-700 rounded-full animate-pulse"></div>
            </div>

            <div className="bg-white dark:bg-gray-800 rounded-2xl shadow-xl p-8 max-w-2xl mx-auto">
              <h2 className="text-2xl font-bold text-gray-800 dark:text-white mb-4">
                Join the Physical AI Community
              </h2>
              <p className="text-gray-600 dark:text-gray-300 mb-6">
                Sign in to access your personalized learning dashboard, track progress, and bookmark chapters.
              </p>
              <div className="flex flex-col sm:flex-row gap-4 justify-center">
                <div className="bg-gradient-to-r from-blue-500 to-purple-600 text-white px-6 py-3 rounded-lg font-medium text-center opacity-50">
                  Loading...
                </div>
                <div className="bg-gradient-to-r from-blue-500 to-purple-600 text-white px-6 py-3 rounded-lg font-medium text-center opacity-50">
                  Loading...
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // If user is authenticated, show the book content
  if (user) {
    return <BookContent />;
  }

  // If user is not authenticated, show the login page
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-900 dark:to-gray-800">
      <Navbar />

      {/* Login Modal */}
      <LoginModal
        isOpen={isLoginModalOpen}
        onClose={() => setIsLoginModalOpen(false)}
      />

      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12">
        <div className="text-center">
          <motion.div
            initial={{ opacity: 0, y: -20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            className="mb-8"
          >
            <h1 className="text-4xl md:text-6xl font-extrabold text-gray-900 dark:text-white mb-4">
              Physical AI & Humanoid Robotics
            </h1>
            <p className="text-xl text-gray-600 dark:text-gray-300 max-w-3xl mx-auto">
              Embodied Intelligence for the Real World - Learn cutting-edge concepts in AI and robotics
            </p>
          </motion.div>

          <motion.div
            initial={{ opacity: 0, scale: 0.9 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 0.5, delay: 0.2 }}
            className="mb-12"
          >
            <RobotAnimation size="lg" animationType="wave" className="mx-auto" />
          </motion.div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 0.4 }}
          >
            <div className="bg-white dark:bg-gray-800 rounded-2xl shadow-xl p-8 max-w-2xl mx-auto">
              <h2 className="text-2xl font-bold text-gray-800 dark:text-white mb-4">
                Join the Physical AI Community
              </h2>
              <p className="text-gray-600 dark:text-gray-300 mb-6">
                Sign in to access your personalized learning dashboard, track progress, and bookmark chapters.
              </p>
              <div className="flex flex-col sm:flex-row gap-4 justify-center">
                <AnimatedLoginButton
                  provider="google"
                  onClick={() => setIsLoginModalOpen(true)}
                >
                  Sign in with Google
                </AnimatedLoginButton>
                <AnimatedLoginButton
                  provider="github"
                  onClick={() => setIsLoginModalOpen(true)}
                >
                  Sign in with GitHub
                </AnimatedLoginButton>
              </div>
            </div>
          </motion.div>

          {/* Features Section */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 0.6 }}
            className="mt-16 grid grid-cols-1 md:grid-cols-3 gap-8"
          >
            <div className="bg-white dark:bg-gray-800 p-6 rounded-xl shadow-lg">
              <div className="text-blue-500 text-3xl mb-4">📚</div>
              <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-2">Structured Learning</h3>
              <p className="text-gray-600 dark:text-gray-300">
                Follow a structured curriculum designed by experts in Physical AI and robotics.
              </p>
            </div>

            <div className="bg-white dark:bg-gray-800 p-6 rounded-xl shadow-lg">
              <div className="text-purple-500 text-3xl mb-4">⚡</div>
              <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-2">Interactive Content</h3>
              <p className="text-gray-600 dark:text-gray-300">
                Engage with interactive elements, quizzes, and practical examples.
              </p>
            </div>

            <div className="bg-white dark:bg-gray-800 p-6 rounded-xl shadow-lg">
              <div className="text-green-500 text-3xl mb-4">👥</div>
              <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-2">Community Access</h3>
              <p className="text-gray-600 dark:text-gray-300">
                Join a community of learners and researchers in Physical AI.
              </p>
            </div>
          </motion.div>
        </div>
      </div>
    </div>
  );
};

export default HomePage;