import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { FaTimes, FaRobot } from 'react-icons/fa';
import { useAuth } from '../context/AuthContext';

interface WelcomePopupProps {
  isOpen: boolean;
  onClose: () => void;
}

const WelcomePopup: React.FC<WelcomePopupProps> = ({ isOpen, onClose }) => {
  const { user } = useAuth();
  const [showOnce, setShowOnce] = useState(false);

  // Show popup only once per session
  useEffect(() => {
    if (isOpen && !showOnce) {
      setShowOnce(true);
    }
  }, [isOpen, showOnce]);

  return (
    <AnimatePresence>
      {isOpen && showOnce && (
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4"
        >
          <motion.div
            initial={{ scale: 0.7, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            exit={{ scale: 0.7, opacity: 0 }}
            className="bg-gradient-to-br from-blue-500 to-purple-600 rounded-2xl shadow-2xl w-full max-w-md overflow-hidden"
          >
            <div className="p-1 bg-gradient-to-r from-yellow-400 to-pink-500">
              <div className="bg-white dark:bg-gray-800 p-8 rounded-xl relative">
                <button 
                  onClick={onClose}
                  className="absolute top-4 right-4 text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-gray-200"
                >
                  <FaTimes size={20} />
                </button>
                
                <div className="text-center">
                  <motion.div
                    animate={{ rotate: [0, 10, -10, 0] }}
                    transition={{ 
                      repeat: Infinity, 
                      repeatType: "reverse",
                      duration: 2
                    }}
                    className="inline-block mb-4"
                  >
                    <FaRobot className="text-5xl text-blue-500 mx-auto" />
                  </motion.div>
                  
                  <h2 className="text-2xl font-bold text-gray-800 dark:text-white mb-3">
                    Welcome to Physical AI, {user?.name || 'Explorer'}!
                  </h2>
                  
                  <p className="text-gray-600 dark:text-gray-300 mb-6">
                    Your personalized learning journey begins now. 
                    Access your dashboard to track progress and bookmark chapters.
                  </p>
                  
                  <div className="flex justify-center space-x-4">
                    <motion.button
                      whileHover={{ scale: 1.05 }}
                      whileTap={{ scale: 0.95 }}
                      onClick={onClose}
                      className="bg-gradient-to-r from-blue-500 to-purple-600 text-white px-6 py-2 rounded-lg font-medium"
                    >
                      Start Learning
                    </motion.button>
                  </div>
                </div>
              </div>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
};

export default WelcomePopup;