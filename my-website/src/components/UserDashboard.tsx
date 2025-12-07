import React, { useState, useEffect } from 'react';
import { motion } from 'framer-motion';
import { FaBook, FaCheckCircle, FaRobot, FaBookmark, FaClock, FaStar } from 'react-icons/fa';
import { useAuth } from '../context/AuthContext';

// Define types for book progress
interface Chapter {
  id: string;
  title: string;
  completed: boolean;
  bookmarked: boolean;
  progress: number; // percentage
}

const UserDashboard: React.FC = () => {
  const { user } = useAuth();
  const [chapters, setChapters] = useState<Chapter[]>([]);
  const [loading, setLoading] = useState(true);

  // Mock data for chapters - in a real app, this would come from a backend
  useEffect(() => {
    // Simulate loading data
    setTimeout(() => {
      setChapters([
        { id: '1', title: 'Introduction to Physical AI', completed: true, bookmarked: true, progress: 100 },
        { id: '2', title: 'Embodied Intelligence', completed: true, bookmarked: false, progress: 100 },
        { id: '3', title: 'Humanoid Robotics Fundamentals', completed: false, bookmarked: true, progress: 65 },
        { id: '4', title: 'Sensory Integration', completed: false, bookmarked: false, progress: 30 },
        { id: '5', title: 'Motor Control Systems', completed: false, bookmarked: true, progress: 10 },
      ]);
      setLoading(false);
    }, 800);
  }, []);

  // Toggle bookmark for a chapter
  const toggleBookmark = (chapterId: string) => {
    setChapters(prev => 
      prev.map(chapter => 
        chapter.id === chapterId 
          ? { ...chapter, bookmarked: !chapter.bookmarked } 
          : chapter
      )
    );
  };

  // Calculate total progress
  const totalProgress = chapters.length 
    ? Math.round(chapters.reduce((sum, chapter) => sum + chapter.progress, 0) / chapters.length)
    : 0;

  // Calculate completed chapters
  const completedChapters = chapters.filter(ch => ch.completed).length;

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-gray-900 dark:to-gray-800 py-8 px-4">
      <div className="max-w-6xl mx-auto">
        {/* User Greeting Section */}
        <motion.div 
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          className="bg-white dark:bg-gray-800 rounded-2xl shadow-xl p-6 mb-8"
        >
          <div className="flex items-center">
            <div className="bg-gradient-to-r from-blue-500 to-purple-600 rounded-full w-16 h-16 flex items-center justify-center text-white text-2xl mr-4">
              <FaRobot />
            </div>
            <div>
              <h1 className="text-2xl font-bold text-gray-800 dark:text-white">
                Welcome back, {user?.name || 'Explorer'}! 
                <span className="ml-2 text-3xl">🤖</span>
              </h1>
              <p className="text-gray-600 dark:text-gray-300">
                Continue your journey in Physical AI and Humanoid Robotics
              </p>
            </div>
          </div>
        </motion.div>

        {/* Progress Overview */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-8">
          <motion.div 
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.1 }}
            className="bg-white dark:bg-gray-800 rounded-2xl shadow-lg p-6"
          >
            <div className="flex items-center">
              <div className="bg-blue-100 dark:bg-blue-900/30 p-3 rounded-full mr-4">
                <FaBook className="text-blue-500 text-xl" />
              </div>
              <div>
                <p className="text-gray-500 dark:text-gray-400 text-sm">Overall Progress</p>
                <p className="text-2xl font-bold text-gray-800 dark:text-white">{totalProgress}%</p>
              </div>
            </div>
            <div className="mt-3 bg-gray-200 dark:bg-gray-700 rounded-full h-2.5">
              <div 
                className="bg-gradient-to-r from-blue-500 to-purple-600 h-2.5 rounded-full" 
                style={{ width: `${totalProgress}%` }}
              ></div>
            </div>
          </motion.div>

          <motion.div 
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2 }}
            className="bg-white dark:bg-gray-800 rounded-2xl shadow-lg p-6"
          >
            <div className="flex items-center">
              <div className="bg-green-100 dark:bg-green-900/30 p-3 rounded-full mr-4">
                <FaCheckCircle className="text-green-500 text-xl" />
              </div>
              <div>
                <p className="text-gray-500 dark:text-gray-400 text-sm">Chapters Completed</p>
                <p className="text-2xl font-bold text-gray-800 dark:text-white">
                  {completedChapters}/{chapters.length}
                </p>
              </div>
            </div>
          </motion.div>

          <motion.div 
            initial={{ opacity: 0, x: 20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.3 }}
            className="bg-white dark:bg-gray-800 rounded-2xl shadow-lg p-6"
          >
            <div className="flex items-center">
              <div className="bg-yellow-100 dark:bg-yellow-900/30 p-3 rounded-full mr-4">
                <FaBookmark className="text-yellow-500 text-xl" />
              </div>
              <div>
                <p className="text-gray-500 dark:text-gray-400 text-sm">Bookmarks</p>
                <p className="text-2xl font-bold text-gray-800 dark:text-white">
                  {chapters.filter(c => c.bookmarked).length}
                </p>
              </div>
            </div>
          </motion.div>
        </div>

        {/* Book Chapters List */}
        <motion.div 
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.4 }}
          className="bg-white dark:bg-gray-800 rounded-2xl shadow-xl overflow-hidden"
        >
          <div className="p-6 border-b border-gray-200 dark:border-gray-700">
            <h2 className="text-xl font-bold text-gray-800 dark:text-white flex items-center">
              <FaBook className="mr-2 text-blue-500" /> Your Book Chapters
            </h2>
          </div>
          
          {loading ? (
            <div className="p-6 flex justify-center">
              <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
            </div>
          ) : (
            <div className="divide-y divide-gray-200 dark:divide-gray-700">
              {chapters.map((chapter, index) => (
                <motion.div 
                  key={chapter.id}
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.1 * index }}
                  className="p-6 hover:bg-gray-50 dark:hover:bg-gray-750 transition-colors duration-200"
                >
                  <div className="flex items-center justify-between">
                    <div className="flex items-center">
                      <div className={`w-10 h-10 rounded-full flex items-center justify-center mr-4 ${
                        chapter.completed 
                          ? 'bg-green-100 dark:bg-green-900/30' 
                          : 'bg-gray-100 dark:bg-gray-700'
                      }`}>
                        {chapter.completed ? (
                          <FaCheckCircle className="text-green-500 text-lg" />
                        ) : (
                          <span className="text-gray-500 dark:text-gray-400 font-medium">
                            {index + 1}
                          </span>
                        )}
                      </div>
                      <div>
                        <h3 className="font-semibold text-gray-800 dark:text-white">{chapter.title}</h3>
                        <div className="flex items-center mt-1">
                          <div className="w-32 bg-gray-200 dark:bg-gray-700 rounded-full h-2 mr-3">
                            <div 
                              className={`h-2 rounded-full ${
                                chapter.progress === 100 
                                  ? 'bg-green-500' 
                                  : 'bg-gradient-to-r from-blue-500 to-purple-600'
                              }`}
                              style={{ width: `${chapter.progress}%` }}
                            ></div>
                          </div>
                          <span className="text-sm text-gray-500 dark:text-gray-400">
                            {chapter.progress}%
                          </span>
                        </div>
                      </div>
                    </div>
                    
                    <div className="flex items-center space-x-3">
                      {chapter.completed && (
                        <motion.div
                          initial={{ scale: 0 }}
                          animate={{ scale: 1 }}
                          className="text-yellow-500"
                        >
                          <FaStar />
                        </motion.div>
                      )}
                      <button 
                        onClick={() => toggleBookmark(chapter.id)}
                        className={`p-2 rounded-full ${
                          chapter.bookmarked
                            ? 'text-yellow-500 bg-yellow-100 dark:bg-yellow-900/30'
                            : 'text-gray-400 hover:text-yellow-500'
                        }`}
                      >
                        <FaBookmark />
                      </button>
                    </div>
                  </div>
                  
                  <div className="mt-3 ml-14">
                    <div className="flex items-center text-sm text-gray-500 dark:text-gray-400">
                      <FaClock className="mr-1" />
                      <span>Estimated time: {(100 - chapter.progress) * 0.5} min</span>
                    </div>
                  </div>
                </motion.div>
              ))}
            </div>
          )}
        </motion.div>
      </div>
    </div>
  );
};

export default UserDashboard;