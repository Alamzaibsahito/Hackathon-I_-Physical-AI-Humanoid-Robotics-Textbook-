import React from 'react';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';

const SignInButton = () => {
  const { isAuthenticated, signOut } = useAuth();

  if (isAuthenticated) {
    return (
      <button className="button button--secondary navbar__item" onClick={signOut}>
        Sign Out
      </button>
    );
  }

  return (
    <Link
      className="button button--secondary navbar__item"
      to="/signup">
      Sign In / Sign Up
    </Link>
  );
};

export default SignInButton;
