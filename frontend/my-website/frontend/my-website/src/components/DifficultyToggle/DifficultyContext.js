import React, { createContext, useContext, useState, useEffect } from 'react';

const DifficultyContext = createContext(null);

export const DifficultyProvider = ({ children }) => {
  const [difficulty, setDifficulty] = useState(() => {
    // Initialize from localStorage or default to 'beginner'
    if (typeof window !== 'undefined') {
      return localStorage.getItem('chapterDifficulty') || 'beginner';
    }
    return 'beginner';
  });

  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem('chapterDifficulty', difficulty);
    }
  }, [difficulty]);

  const toggleDifficulty = () => {
    setDifficulty((prevDifficulty) =>
      prevDifficulty === 'beginner' ? 'advanced' : 'beginner'
    );
  };

  return (
    <DifficultyContext.Provider value={{ difficulty, setDifficulty, toggleDifficulty }}>
      {children}
    </DifficultyContext.Provider>
  );
};

export const useDifficulty = () => useContext(DifficultyContext);
