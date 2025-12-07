import React from 'react';
import { motion } from 'framer-motion';
import { FcGoogle } from 'react-icons/fc';
import { FaGithub, FaSignInAlt } from 'react-icons/fa';

interface AnimatedLoginButtonProps {
  provider: 'google' | 'github' | 'login';
  onClick: () => void;
  disabled?: boolean;
  children?: React.ReactNode;
}

const AnimatedLoginButton: React.FC<AnimatedLoginButtonProps> = ({ 
  provider, 
  onClick, 
  disabled = false,
  children
}) => {
  const getButtonConfig = () => {
    switch(provider) {
      case 'google':
        return {
          bg: 'bg-gradient-to-r from-blue-500 to-blue-600 hover:from-blue-600 hover:to-blue-700',
          icon: <FcGoogle className="text-xl" />
        };
      case 'github':
        return {
          bg: 'bg-gradient-to-r from-gray-700 to-gray-900 hover:from-gray-800 hover:to-black',
          icon: <FaGithub className="text-xl text-white" />
        };
      case 'login':
      default:
        return {
          bg: 'bg-gradient-to-r from-purple-600 to-indigo-700 hover:from-purple-700 hover:to-indigo-800',
          icon: <FaSignInAlt className="text-xl" />
        };
    }
  };

  const config = getButtonConfig();
  
  return (
    <motion.button
      whileHover={{ 
        scale: 1.03,
        y: -2,
        boxShadow: "0 10px 25px rgba(0,0,0,0.2)"
      }}
      whileTap={{ scale: 0.98 }}
      animate={{
        boxShadow: [
          "0px 4px 10px rgba(0,0,0,0.1)",
          "0px 6px 15px rgba(0,0,0,0.15)",
          "0px 4px 10px rgba(0,0,0,0.1)"
        ]
      }}
      transition={{
        duration: 2,
        repeat: Infinity,
        repeatType: "reverse"
      }}
      className={`
        flex items-center justify-center w-full py-3 px-4 rounded-xl
        font-medium text-white shadow-lg
        transition-all duration-300
        ${config.bg}
        ${disabled ? 'opacity-50 cursor-not-allowed' : 'hover:shadow-xl'}
      `}
      onClick={onClick}
      disabled={disabled}
    >
      <span className="mr-3">
        {config.icon}
      </span>
      {children || `Continue with ${provider.charAt(0).toUpperCase() + provider.slice(1)}`}
    </motion.button>
  );
};

export default AnimatedLoginButton;