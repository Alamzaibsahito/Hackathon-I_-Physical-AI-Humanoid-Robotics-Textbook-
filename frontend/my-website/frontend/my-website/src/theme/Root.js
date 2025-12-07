import React from 'react';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import { DifficultyProvider } from '@site/src/components/DifficultyToggle/DifficultyContext';

// Default implementation of the Root component, just renders children.
// We wrap it with our AuthProvider and DifficultyProvider to make contexts available globally.
function Root({ children }) {
  return (
    <AuthProvider>
      <DifficultyProvider>
        {children}
      </DifficultyProvider>
    </AuthProvider>
  );
}

export default Root;
