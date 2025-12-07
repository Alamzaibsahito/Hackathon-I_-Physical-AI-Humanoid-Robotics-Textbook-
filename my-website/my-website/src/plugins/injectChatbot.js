import React from 'react';
import { createRoot } from 'react-dom/client';
import Chatbot from '../components/Chatbot';

export default function injectChatbot() {
  if (typeof document !== 'undefined') {
    let chatbotContainer = document.getElementById('chatbot-root');
    if (!chatbotContainer) {
      chatbotContainer = document.createElement('div');
      chatbotContainer.id = 'chatbot-root';
      document.body.appendChild(chatbotContainer);
    }

    const root = createRoot(chatbotContainer);
    root.render(<Chatbot />);
  }
}