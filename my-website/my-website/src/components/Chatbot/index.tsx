import React, { useState, useEffect, useRef } from 'react';
import { getSessionId, chatRAG, ChatResponse, Document } from '../../utils/api';
import { useColorMode } from '@docusaurus/theme-common';
import './chatbot.css'; // Ensure this import is present

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{ type: 'user' | 'bot'; text: string; sources?: Document[] }[]>([]);
  const [input, setInput] = useState('');
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { colorMode } = useColorMode(); // Get current theme

  useEffect(() => {
    const sId = getSessionId();
    setSessionId(sId);
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInput(e.target.value);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || !sessionId) return;

    const userMessage = input;
    setInput('');
    setMessages((prev) => [...prev, { type: 'user', text: userMessage }]);
    setIsLoading(true);

    let selectedText: string | null = null;
    if (typeof window !== 'undefined') {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        selectedText = selection.toString();
      }
    }

    try {
      const response: ChatResponse = await chatRAG(sessionId, userMessage, selectedText);
      setMessages((prev) => [...prev, { type: 'bot', text: response.response, sources: response.sources }]);
    } catch (error) {
      console.error('Chatbot API error:', error);
      setMessages((prev) => [...prev, { type: 'bot', text: 'Sorry, something went wrong. Please try again.' }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`chatbot-container ${colorMode}`}>
      <div className="chatbot-bubble" onClick={toggleChatbot}>
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M7.99998 12C7.99998 12.8284 7.32841 13.5 6.49998 13.5C5.67155 13.5 4.99998 12.8284 4.99998 12C4.99998 11.1716 5.67155 10.5 6.49998 10.5C7.32841 10.5 7.99998 11.1716 7.99998 12Z" fill="currentColor"/>
          <path d="M12 12C12 12.8284 11.3284 13.5 10.5 13.5C9.67157 13.5 9 12.8284 9 12C9 11.1716 9.67157 10.5 10.5 10.5C11.3284 10.5 12 11.1716 12 12Z" fill="currentColor"/>
          <path d="M16 12C16 12.8284 15.3284 13.5 14.5 13.5C13.6716 13.5 13 12.8284 13 12C13 11.1716 13.6716 10.5 14.5 10.5C15.3284 10.5 16 11.1716 16 12Z" fill="currentColor"/>
          <path fillRule="evenodd" clipRule="evenodd" d="M12 2C6.47715 2 2 6.47715 2 12C2 13.9961 2.58557 15.8643 3.61905 17.4312L2.02929 21.0105C1.72895 21.7018 2.30231 22.4839 3.03781 22.3787L7.49524 21.7585C9.07923 22.4633 10.4687 22.9091 12 23C17.5228 23 22 18.5228 22 13C22 7.47715 17.5228 2 12 2ZM4 12C4 7.58172 7.58172 4 12 4C16.4183 4 20 7.58172 20 12C20 16.4183 16.4183 20 12 20C10.5313 20 9.14175 19.5638 7.90422 18.7844L4.85355 19.3464L5.61864 16.9079C4.54288 15.421 4 13.7749 4 12Z" fill="currentColor"/>
        </svg>
      </div>
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Physical AI Chatbot</h3>
            <button className="close-button" onClick={toggleChatbot}>×</button>
          </div>
          <div className="chatbot-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.type}`}>
                <p>{msg.text}</p>
                {msg.type === 'bot' && msg.sources && msg.sources.length > 0 && (
                  <div className="message-sources">
                    <strong>Sources:</strong>
                    <ul>
                      {msg.sources.map((source, srcIndex) => (
                        <li key={srcIndex}>
                          {source.metadata?.docusaurus_url ? (
                            <a href={source.metadata.docusaurus_url} target="_blank" rel="noopener noreferrer">
                              {source.metadata.docusaurus_title || `Source ${srcIndex + 1}`}
                            </a>
                          ) : (
                            `Source ${srcIndex + 1}: ${source.content.substring(0, 50)}...`
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className="message bot">
                <p>Thinking...</p>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <input
              type="text"
              value={input}
              onChange={handleInputChange}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
    </div>
  );
};

export default Chatbot;