// API utility for chatbot
const BACKEND_BASE_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:5000';

export const chatApi = {
  async query(message, history = []) {
    try {
      const response = await fetch(`${BACKEND_BASE_URL}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          history
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Chat API error:', error);
      throw error;
    }
  },

  async healthCheck() {
    try {
      const response = await fetch(`${BACKEND_BASE_URL}/chat/health`);
      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }
};