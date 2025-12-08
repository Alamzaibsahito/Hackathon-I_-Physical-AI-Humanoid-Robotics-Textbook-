import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '../components/Chatbot';

const Layout = (props) => {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <Chatbot />
    </OriginalLayout>
  );
};

export default Layout;