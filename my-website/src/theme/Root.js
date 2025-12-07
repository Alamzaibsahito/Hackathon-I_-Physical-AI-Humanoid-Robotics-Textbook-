import React, { useState, useEffect } from 'react';
import { AuthProvider } from '../context/AuthContext';

// Root component that wraps the entire app
export default function Root({ children }) {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // During SSR, return children without auth wrapper
  if (!isClient) {
    return <>{children}</>;
  }

  // On client, wrap with auth provider
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}