import React from 'react';
import { useDifficulty } from './DifficultyContext';
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const PersonalizedContent = ({ children, difficulty: requiredDifficulty }) => {
  const { difficulty: currentDifficulty } = useDifficulty();
  const { isAuthenticated } = useAuth();

  // Only show personalized content if authenticated
  if (!isAuthenticated) {
    return null; // Or render a message prompting login
  }

  if (currentDifficulty === requiredDifficulty) {
    return <>{children}</>;
  }

  return null;
};

export default PersonalizedContent;
