import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// Default implementation, that you might want to customize
function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}

export default Root;
