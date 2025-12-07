import { useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';

export const useProtectedRoute = (redirectTo = '/') => {
  const { user, loading } = useAuth();
  const location = useLocation();
  const history = useHistory();

  useEffect(() => {
    if (!loading && !user) {
      // Store the attempted path so we can redirect back after login
      localStorage.setItem('redirectAfterLogin', location.pathname);
      history.push(redirectTo);
    }
  }, [user, loading, location.pathname, redirectTo, history]);

  return { user, loading };
};