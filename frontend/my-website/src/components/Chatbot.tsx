import React, { useState } from 'react';

const FASTAPI_BACKEND_URL = 'http://localhost:8000'; // TODO: Make this configurable via environment variable

interface ChatMessage {
  id: number;
  text: string;
  sender: 'user' | 'bot';
}

interface ChatbotProps {
  initialContext?: string; // Optional prop to provide initial selected text context
}

const Chatbot: React.FC<ChatbotProps> = ({ initialContext }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSendMessage = async () => {
    if (input.trim() === '') return;

    const newUserMessage: ChatMessage = { id: messages.length + 1, text: input, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${FASTAPI_BACKEND_URL}/rag/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: input, context: initialContext }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const newBotMessage: ChatMessage = { id: messages.length + 2, text: data.answer, sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, newBotMessage]);
    } catch (error) {
      console.error("Error sending message:", error);
      const errorMessage: ChatMessage = { id: messages.length + 2, text: "Error: Could not get a response.", sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ border: '1px solid #ccc', borderRadius: '8px', padding: '16px', maxWidth: '400px', margin: '0 auto' }}>
      <h3>AI Chatbot</h3>
      <div style={{ height: '300px', overflowY: 'scroll', border: '1px solid #eee', padding: '8px', marginBottom: '16px' }}>
        {messages.map((msg) => (
          <div key={msg.id} style={{ textAlign: msg.sender === 'user' ? 'right' : 'left', margin: '4px 0' }}>
            <span style={{
              display: 'inline-block',
              padding: '8px 12px',
              borderRadius: '18px',
              backgroundColor: msg.sender === 'user' ? '#007bff' : '#e2e6ea',
              color: msg.sender === 'user' ? 'white' : 'black',
            }}>
              {msg.text}
            </span>
          </div>
        ))}
        {loading && <p>Thinking...</p>}
      </div>
      <div style={{ display: 'flex' }}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => {
            if (e.key === 'Enter') {
              handleSendMessage();
            }
          }}
          placeholder="Ask a question..."
          style={{ flexGrow: 1, padding: '8px', borderRadius: '4px', border: '1px solid #ccc' }}
          disabled={loading}
        />
        <button onClick={handleSendMessage} style={{ marginLeft: '8px', padding: '8px 16px', borderRadius: '4px', border: 'none', backgroundColor: '#007bff', color: 'white', cursor: 'pointer' }} disabled={loading}>
          Send
        </button>
      </div>
    </div>
  );
};

export default Chatbot;
