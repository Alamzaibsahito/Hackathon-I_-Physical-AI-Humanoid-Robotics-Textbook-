import React, { useState } from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { useDoc } from '@docusaurus/theme-common/internal';
import { useCurrentLocale } from '@docusaurus/theme-common';

const TranslateButton = () => {
  const { isAuthenticated } = useAuth();
  const { frontMatter } = useDoc();
  const currentLocale = useCurrentLocale();
  const [translatedContent, setTranslatedContent] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Only show the button if authenticated, not already in Urdu locale, and on a doc page
  if (!isAuthenticated || currentLocale === 'ur' || !frontMatter) {
    return null;
  }

  const handleTranslate = async () => {
    setLoading(true);
    setError(null);
    setTranslatedContent(null);

    const docContent = document.querySelector('.markdown').innerText; // Get current page text

    try {
      const response = await fetch(`${process.env.FRONTEND_API_URL || 'http://localhost:8000'}/api/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: docContent, target_language: 'ur' }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setTranslatedContent(data.translated_text);
    } catch (err) {
      console.error('Translation failed:', err);
      setError('Failed to translate content.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{ marginTop: '2rem', textAlign: 'right' }}>
      <button
        onClick={handleTranslate}
        disabled={loading}
        style={{
          background: loading ? '#cccccc' : '#17a2b8',
          color: 'white',
          border: 'none',
          padding: '0.5rem 1rem',
          borderRadius: '5px',
          cursor: 'pointer',
          fontWeight: 'bold',
          marginRight: '0.5rem',
        }}
      >
        {loading ? 'Translating...' : 'Translate to Urdu'}
      </button>
      {error && <p style={{ color: 'red' }}>{error}</p>}
      {translatedContent && (
        <div
          style={{
            marginTop: '1rem',
            padding: '1rem',
            border: '1px solid #17a2b8',
            borderRadius: '5px',
            backgroundColor: '#e0f7fa',
            textAlign: 'left',
          }}
        >
          <h3>Urdu Translation:</h3>
          <p>{translatedContent}</p>
        </div>
      )}
    </div>
  );
};

export default TranslateButton;
