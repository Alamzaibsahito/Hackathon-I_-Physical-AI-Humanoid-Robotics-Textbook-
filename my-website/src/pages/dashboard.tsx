import React, { useState, useEffect } from 'react';
import { Redirect, useLocation } from '@docusaurus/router';
import { useAuth } from '../context/AuthContext';
import UserDashboard from '../components/UserDashboard';

const DashboardPage = () => {
  const { user, loading } = useAuth();
  const [isClient, setIsClient] = useState(false);
  const location = useLocation();

  useEffect(() => {
    setIsClient(true);
  }, []);

  // During SSR, render a minimal state
  if (!isClient) {
    return (
      <div className="min-h-[50vh] flex items-center justify-center">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
      </div>
    );
  }

  // During client-side loading
  if (loading) {
    return (
      <div className="min-h-[50vh] flex items-center justify-center">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
      </div>
    );
  }

  // If user is not authenticated
  if (!user) {
    // Store the attempted path so we can redirect back after login
    if (typeof window !== 'undefined') {
      localStorage.setItem('redirectAfterLogin', location.pathname);
    }
    return <Redirect to="/" />;
  }

  // User is authenticated, render dashboard
  return <UserDashboard />;
};

export default DashboardPage;