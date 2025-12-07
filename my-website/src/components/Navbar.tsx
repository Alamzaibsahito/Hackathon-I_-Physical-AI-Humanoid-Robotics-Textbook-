import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import LoginModal from '../components/LoginModal';
import WelcomePopup from '../components/WelcomePopup';
import { motion, AnimatePresence } from 'framer-motion';
import { FaUser, FaSignOutAlt, FaBook, FaRobot, FaBars, FaTimes } from 'react-icons/fa';

const Navbar = () => {
  const { user, logout } = useAuth();
  const [isLoginModalOpen, setIsLoginModalOpen] = useState(false);
  const [isWelcomePopupOpen, setIsWelcomePopupOpen] = useState(false);
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [isScrolled, setIsScrolled] = useState(false);

  // Show welcome popup after login
  useEffect(() => {
    if (user && !isWelcomePopupOpen && localStorage.getItem('showWelcome') !== 'false') {
      setIsWelcomePopupOpen(true);
      localStorage.setItem('showWelcome', 'false'); // Don't show again in this session
    }
  }, [user, isWelcomePopupOpen]);

  // Handle scroll for navbar effects
  useEffect(() => {
    const handleScroll = () => {
      setIsScrolled(window.scrollY > 10);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  // Navigation items
  const navItems = [
    { name: 'Textbook', href: '/preface/intro' },
    { name: 'Docs', href: '/docs' },
    { name: 'Blog', href: '/blog' },
  ];

  return (
    <>
      {/* Welcome Popup */}
      <WelcomePopup 
        isOpen={isWelcomePopupOpen} 
        onClose={() => setIsWelcomePopupOpen(false)} 
      />

      {/* Login Modal */}
      <LoginModal 
        isOpen={isLoginModalOpen} 
        onClose={() => setIsLoginModalOpen(false)} 
      />

      {/* Floating Navbar */}
      <AnimatePresence>
        <motion.nav 
          className={`fixed top-0 left-0 right-0 z-50 transition-all duration-300 ${
            isScrolled ? 'bg-white/90 dark:bg-gray-900/90 backdrop-blur-sm shadow-md' : 'bg-transparent'
          }`}
          initial={{ y: -100 }}
          animate={{ y: 0 }}
          transition={{ duration: 0.5 }}
        >
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="flex justify-between items-center h-16">
              {/* Logo */}
              <div className="flex items-center">
                <div className="flex-shrink-0 flex items-center">
                  <FaRobot className="h-8 w-8 text-blue-500 mr-2" />
                  <span className="text-xl font-bold text-gray-900 dark:text-white">Physical AI</span>
                </div>
                
                {/* Desktop Navigation - Hidden on mobile */}
                <div className="hidden md:ml-10 md:flex md:space-x-8">
                  {navItems.map((item) => (
                    <a
                      key={item.name}
                      href={item.href}
                      className="text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 px-1 pt-1 text-sm font-medium transition-colors duration-200"
                    >
                      {item.name}
                    </a>
                  ))}
                </div>
              </div>

              {/* Right side - user actions */}
              <div className="flex items-center">
                {/* User dashboard or login button - Hidden on mobile */}
                <div className="hidden md:flex items-center space-x-4">
                  {user ? (
                    <>
                      <motion.a
                        href="/dashboard"
                        whileHover={{ scale: 1.05 }}
                        whileTap={{ scale: 0.95 }}
                        className="flex items-center text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 text-sm font-medium"
                      >
                        <FaBook className="mr-1" /> Dashboard
                      </motion.a>
                      
                      <div className="relative group">
                        <motion.button
                          whileHover={{ scale: 1.05 }}
                          whileTap={{ scale: 0.95 }}
                          className="flex items-center text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 text-sm font-medium"
                        >
                          <div className="flex items-center justify-center w-8 h-8 rounded-full bg-blue-100 dark:bg-blue-900 text-blue-600 dark:text-blue-300 mr-2">
                            <FaUser />
                          </div>
                          {user.name}
                        </motion.button>
                        
                        <div className="absolute right-0 mt-2 w-48 rounded-md shadow-lg bg-white dark:bg-gray-800 ring-1 ring-black ring-opacity-5 hidden group-hover:block z-50">
                          <div className="py-1">
                            <a 
                              href="/dashboard" 
                              className="block px-4 py-2 text-sm text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700"
                            >
                              Your Dashboard
                            </a>
                            <button
                              onClick={logout}
                              className="block w-full text-left px-4 py-2 text-sm text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700"
                            >
                              Sign out
                            </button>
                          </div>
                        </div>
                      </div>
                    </>
                  ) : (
                    <motion.button
                      whileHover={{ scale: 1.05 }}
                      whileTap={{ scale: 0.95 }}
                      onClick={() => setIsLoginModalOpen(true)}
                      className="bg-gradient-to-r from-blue-500 to-purple-600 text-white px-4 py-2 rounded-lg text-sm font-medium shadow-md hover:shadow-lg transition-shadow duration-200"
                    >
                      Sign In
                    </motion.button>
                  )}
                </div>

                {/* Mobile menu button */}
                <div className="md:hidden ml-4">
                  <button
                    onClick={() => setIsMenuOpen(!isMenuOpen)}
                    className="inline-flex items-center justify-center p-2 rounded-md text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 focus:outline-none"
                  >
                    {isMenuOpen ? <FaTimes size={20} /> : <FaBars size={20} />}
                  </button>
                </div>
              </div>
            </div>

            {/* Mobile menu */}
            <AnimatePresence>
              {isMenuOpen && (
                <motion.div
                  initial={{ opacity: 0, height: 0 }}
                  animate={{ opacity: 1, height: 'auto' }}
                  exit={{ opacity: 0, height: 0 }}
                  className="md:hidden"
                >
                  <div className="px-2 pt-2 pb-3 space-y-1">
                    {navItems.map((item) => (
                      <a
                        key={item.name}
                        href={item.href}
                        className="text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 block px-3 py-2 rounded-md text-base font-medium"
                        onClick={() => setIsMenuOpen(false)}
                      >
                        {item.name}
                      </a>
                    ))}
                    <div className="border-t border-gray-200 dark:border-gray-700 my-2"></div>
                    {user ? (
                      <>
                        <a 
                          href="/dashboard" 
                          className="text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 block px-3 py-2 rounded-md text-base font-medium flex items-center"
                        >
                          <FaBook className="mr-2" /> Dashboard
                        </a>
                        <button
                          onClick={() => {
                            logout();
                            setIsMenuOpen(false);
                          }}
                          className="w-full text-left text-gray-700 dark:text-gray-300 hover:text-blue-600 dark:hover:text-blue-400 block px-3 py-2 rounded-md text-base font-medium flex items-center"
                        >
                          <FaSignOutAlt className="mr-2" /> Sign out
                        </button>
                      </>
                    ) : (
                      <button
                        onClick={() => {
                          setIsLoginModalOpen(true);
                          setIsMenuOpen(false);
                        }}
                        className="w-full text-left bg-gradient-to-r from-blue-500 to-purple-600 text-white px-3 py-2 rounded-md text-base font-medium"
                      >
                        Sign In
                      </button>
                    )}
                  </div>
                </motion.div>
              )}
            </AnimatePresence>
          </div>
        </motion.nav>
      </AnimatePresence>

      {/* Spacer to account for fixed navbar */}
      <div className="h-16"></div>
    </>
  );
};

export default Navbar;