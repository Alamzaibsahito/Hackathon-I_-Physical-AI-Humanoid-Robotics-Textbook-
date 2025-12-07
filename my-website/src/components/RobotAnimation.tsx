import React from 'react';
import { motion } from 'framer-motion';

interface RobotAnimationProps {
  size?: 'sm' | 'md' | 'lg';
  animationType?: 'idle' | 'wave' | 'bounce';
  className?: string;
}

const RobotAnimation: React.FC<RobotAnimationProps> = ({ 
  size = 'md', 
  animationType = 'idle',
  className = '' 
}) => {
  // Size mappings
  const sizeClasses = {
    sm: 'w-16 h-16',
    md: 'w-24 h-24',
    lg: 'w-32 h-32'
  };

  // Animation variants
  const animationVariants = {
    idle: {
      y: [0, -5, 0],
      transition: {
        duration: 3,
        repeat: Infinity,
        ease: "easeInOut"
      }
    },
    wave: {
      rotate: [0, 20, 0, -20, 0],
      transition: {
        duration: 2,
        repeat: Infinity,
        repeatType: "loop" as const
      }
    },
    bounce: {
      y: [0, -20, 0],
      transition: {
        duration: 0.8,
        repeat: Infinity,
        ease: "easeInOut"
      }
    }
  };

  return (
    <motion.div
      className={`${sizeClasses[size]} ${className}`}
      variants={animationVariants}
      animate={animationType}
    >
      <svg 
        viewBox="0 0 100 100" 
        className="w-full h-full"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Robot body */}
        <rect 
          x="30" 
          y="30" 
          width="40" 
          height="40" 
          rx="5" 
          fill="#4F46E5" 
          stroke="#312E81" 
          strokeWidth="2"
        />
        
        {/* Robot head */}
        <rect 
          x="35" 
          y="15" 
          width="30" 
          height="20" 
          rx="3" 
          fill="#6366F1" 
          stroke="#312E81" 
          strokeWidth="2"
        />
        
        {/* Eyes */}
        <circle cx="43" cy="25" r="3" fill="white" />
        <circle cx="57" cy="25" r="3" fill="white" />
        
        {/* Eye pupils */}
        <circle cx="43" cy="25" r="1.5" fill="#1E1B4B" />
        <circle cx="57" cy="25" r="1.5" fill="#1E1B4B" />
        
        {/* Mouth */}
        <path 
          d="M45 30 Q50 33 55 30" 
          stroke="white" 
          strokeWidth="1.5" 
          fill="none"
        />
        
        {/* Arms */}
        <line 
          x1="25" 
          y1="40" 
          x2="15" 
          y2="40" 
          stroke="#6366F1" 
          strokeWidth="4" 
          strokeLinecap="round"
        />
        <line 
          x1="75" 
          y1="40" 
          x2="85" 
          y2="40" 
          stroke="#6366F1" 
          strokeWidth="4" 
          strokeLinecap="round"
        />
        
        {/* Legs */}
        <line 
          x1="40" 
          y1="70" 
          x2="40" 
          y2="80" 
          stroke="#6366F1" 
          strokeWidth="4" 
          strokeLinecap="round"
        />
        <line 
          x1="60" 
          y1="70" 
          x2="60" 
          y2="80" 
          stroke="#6366F1" 
          strokeWidth="4" 
          strokeLinecap="round"
        />
        
        {/* Antenna */}
        <line 
          x1="50" 
          y1="15" 
          x2="50" 
          y2="5" 
          stroke="#6366F1" 
          strokeWidth="2" 
          strokeLinecap="round"
        />
        <circle cx="50" cy="3" r="2" fill="#EF4444" />
      </svg>
    </motion.div>
  );
};

export default RobotAnimation;