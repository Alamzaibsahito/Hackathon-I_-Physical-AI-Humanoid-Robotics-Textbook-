import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  // State to track if background questions have been answered
  const [hasBackgroundInfo, setHasBackgroundInfo] = useState(false);

  useEffect(() => {
    // On mount, try to load user from local storage or check session with backend
    const token = localStorage.getItem('better-auth-token');
    if (token) {
      // In a real app, validate token with backend
      // For now, simulate a logged-in user
      setIsAuthenticated(true);
      setUser({ email: 'simulated@example.com', id: 'sim-user-123' });
      // Also load background info status
      setHasBackgroundInfo(localStorage.getItem('hasBackgroundInfo') === 'true');
    }
    setLoading(false);
  }, []);

  const signIn = async (email, password) => {
    // Simulate API call to better-auth.com / FastAPI backend
    console.log('Simulating sign in...', email, password);
    return new Promise(resolve => {
      setTimeout(() => {
        localStorage.setItem('better-auth-token', 'mock-jwt-token');
        setIsAuthenticated(true);
        setUser({ email: email, id: 'sim-user-123' });
        const backgroundSet = localStorage.getItem('hasBackgroundInfo') === 'true';
        setHasBackgroundInfo(backgroundSet);
        resolve({ success: true, needsBackground: !backgroundSet });
      }, 1000);
    });
  };

  const signUp = async (email, password) => {
    // Simulate API call to better-auth.com / FastAPI backend
    console.log('Simulating sign up...', email, password);
    return new Promise(resolve => {
      setTimeout(() => {
        localStorage.setItem('better-auth-token', 'mock-jwt-token');
        setIsAuthenticated(true);
        setUser({ email: email, id: 'sim-user-123' });
        setHasBackgroundInfo(false); // New user needs to fill background
        localStorage.setItem('hasBackgroundInfo', 'false');
        resolve({ success: true, needsBackground: true });
      }, 1000);
    });
  };

  const signOut = () => {
    localStorage.removeItem('better-auth-token');
    localStorage.removeItem('hasBackgroundInfo');
    setIsAuthenticated(false);
    setUser(null);
    setHasBackgroundInfo(false);
  };

  const submitBackgroundInfo = async (software_experience, hardware_experience) => {
    // Simulate API call to backend/api/auth/signup-complete
    console.log('Submitting background info...', { software_experience, hardware_experience });
    try {
      const response = await fetch(`${process.env.FRONTEND_API_URL || 'http://localhost:8000'}/api/auth/signup-complete`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ software_experience, hardware_experience }),
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      console.log('Background info submission response:', data);
      setHasBackgroundInfo(true);
      localStorage.setItem('hasBackgroundInfo', 'true');
      return { success: true };
    } catch (error) {
      console.error('Failed to submit background info:', error);
      return { success: false, error: error.message };
    }
  };

  return (
    <AuthContext.Provider value={{
      isAuthenticated,
      user,
      loading,
      hasBackgroundInfo,
      signIn,
      signUp,
      signOut,
      submitBackgroundInfo,
    }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
