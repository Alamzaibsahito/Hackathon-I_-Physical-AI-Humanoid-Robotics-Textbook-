import React from 'react';
import SignInButton from '@site/src/components/Auth/SignInButton';

// This component is a custom NavbarItem type that renders our SignInButton.
const CustomAuthNavbarItem = (props) => {
  return <SignInButton {...props} />;
};

export default CustomAuthNavbarItem;
