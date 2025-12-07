import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

// Define the user type
interface User {
  id: string;
  name: string;
  email: string;
  picture?: string;
  provider: string; // 'google' or 'github'
}

// Define the auth context type
interface AuthContextType {
  user: User | null;
  login: (user: User) => void;
  logout: () => void;
  loading: boolean;
  googleLogin: () => void;
  githubLogin: () => void;
}

// Create the auth context with a default value
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Auth provider component
export const AuthProvider = ({ children }: { children: ReactNode }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Load user from localStorage on initial render
  useEffect(() => {
    // Check if running in browser environment
    if (typeof window !== 'undefined') {
      const storedUser = localStorage.getItem('user');
      if (storedUser) {
        try {
          setUser(JSON.parse(storedUser));
        } catch (error) {
          console.error('Error parsing stored user:', error);
        }
      }
    }
    setLoading(false);
  }, []);

  // Login function
  const login = (userData: User) => {
    // Check if running in browser environment
    if (typeof window !== 'undefined') {
      setUser(userData);
      localStorage.setItem('user', JSON.stringify(userData));
    }
  };

  // Logout function
  const logout = () => {
    // Check if running in browser environment
    if (typeof window !== 'undefined') {
      setUser(null);
      localStorage.removeItem('user');
      // Also clear OAuth tokens if needed
      localStorage.removeItem('oauth_token');
    }
  };

  // Google OAuth login (placeholder - would connect to backend in real implementation)
  const googleLogin = () => {
    // In a real implementation, this would redirect to your backend for OAuth
    // For now, we'll simulate a successful login
    const mockUser = {
      id: `google_${Date.now()}`,
      name: 'Demo Google User',
      email: 'google-demo@example.com',
      picture: 'https://lh3.googleusercontent.com/a-/AOh14Gg',
      provider: 'google'
    };
    login(mockUser);
  };

  // GitHub OAuth login (placeholder - would connect to backend in real implementation)
  const githubLogin = () => {
    // In a real implementation, this would redirect to your backend for OAuth
    // For now, we'll simulate a successful login
    const mockUser = {
      id: `github_${Date.now()}`,
      name: 'Demo GitHub User',
      email: 'github-demo@example.com',
      picture: 'https://github.com/identicons',
      provider: 'github'
    };
    login(mockUser);
  };

  const value = {
    user,
    login,
    logout,
    loading,
    googleLogin,
    githubLogin
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context with SSR safety
export const useAuth = () => {
  const context = useContext(AuthContext);

  // If context is undefined, it means it's being used outside of AuthProvider
  // In SSR, context will be undefined, so we return a default state
  if (context === undefined) {
    // Check if we're in the browser
    if (typeof window !== 'undefined' && typeof window.document !== 'undefined') {
      // We're on the client but outside of AuthProvider
      throw new Error('useAuth must be used within an AuthProvider');
    }
    // We're on the server, return default values
    return {
      user: null,
      login: () => {},
      logout: () => {},
      loading: true,
      googleLogin: () => {},
      githubLogin: () => {}
    };
  }

  return context;
};