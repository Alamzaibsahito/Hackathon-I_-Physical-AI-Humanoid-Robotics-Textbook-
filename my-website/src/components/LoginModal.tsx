import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { useGoogleLogin } from '@react-oauth/google';
import { FaTimes, FaRobot } from 'react-icons/fa';
import { useAuth } from '../context/AuthContext';
import AnimatedLoginButton from './AnimatedLoginButton';
import Confetti from 'react-confetti';

interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const LoginModal: React.FC<LoginModalProps> = ({ isOpen, onClose }) => {
  const [showConfetti, setShowConfetti] = useState(false);
  const [isClient, setIsClient] = useState(false);
  const { googleLogin: contextGoogleLogin, githubLogin, user } = useAuth();

  // Check if we're on the client side
  useEffect(() => {
    setIsClient(true);
  }, []);

  // Effect to handle successful login
  useEffect(() => {
    if (user) {
      // Show confetti effect
      setShowConfetti(true);
      setTimeout(() => setShowConfetti(false), 5000);

      // Close the modal after successful login
      setTimeout(onClose, 1500);
    }
  }, [user, onClose]);

  // Google OAuth login handler - only initialize on client
  const handleGoogleLogin = () => {
    contextGoogleLogin();
  };

  // GitHub OAuth login handler
  const handleGitHubLogin = () => {
    githubLogin();
  };

  return (
    <AnimatePresence>
      {isOpen && (
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4"
          onClick={onClose}
        >
          {/* Confetti effect for successful login */}
          {showConfetti && <Confetti recycle={false} numberOfPieces={500} />}

          <motion.div
            initial={{ scale: 0.8, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            exit={{ scale: 0.8, opacity: 0 }}
            className="bg-white dark:bg-gray-800 rounded-2xl shadow-2xl w-full max-w-md overflow-hidden"
            onClick={(e) => e.stopPropagation()}
          >
            <div className="p-1 bg-gradient-to-r from-blue-400 via-purple-500 to-pink-500">
              <div className="bg-white dark:bg-gray-800 p-6 rounded-xl">
                <div className="flex justify-between items-center mb-6">
                  <h2 className="text-2xl font-bold text-gray-800 dark:text-white flex items-center">
                    <FaRobot className="mr-2 text-blue-500" />
                    Welcome to Physical AI
                  </h2>
                  <button
                    onClick={onClose}
                    className="text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-gray-200"
                  >
                    <FaTimes size={20} />
                  </button>
                </div>

                <p className="text-gray-600 dark:text-gray-300 mb-6 text-center">
                  Sign in to access your personalized learning dashboard
                </p>

                <div className="space-y-4">
                  <AnimatedLoginButton
                    provider="google"
                    onClick={handleGoogleLogin}
                  >
                    Continue with Google
                  </AnimatedLoginButton>
                  <AnimatedLoginButton
                    provider="github"
                    onClick={handleGitHubLogin}
                  >
                    Continue with GitHub
                  </AnimatedLoginButton>
                </div>

                <div className="mt-6 text-center">
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    By signing in, you agree to our Terms of Service and Privacy Policy
                  </p>
                </div>
              </div>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
};

export default LoginModal;