import React from 'react';
import { useDifficulty } from './DifficultyContext';
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const DifficultyToggle = () => {
  const { difficulty, toggleDifficulty } = useDifficulty();
  const { isAuthenticated } = useAuth();

  // Only show the toggle if the user is authenticated
  if (!isAuthenticated) {
    return null;
  }

  return (
    <div style={{ marginBottom: '1rem', textAlign: 'right' }}>
      <button
        onClick={toggleDifficulty}
        style={{
          background: 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          padding: '0.5rem 1rem',
          borderRadius: '5px',
          cursor: 'pointer',
          fontWeight: 'bold',
        }}
      >
        Switch to {difficulty === 'beginner' ? 'Advanced' : 'Beginner'} Mode
      </button>
    </div>
  );
};

export default DifficultyToggle;
