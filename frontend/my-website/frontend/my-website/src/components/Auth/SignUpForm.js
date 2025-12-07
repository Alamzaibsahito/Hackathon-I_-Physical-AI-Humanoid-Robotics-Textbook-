import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';

const SignUpForm = () => {
  const { signIn, signUp, submitBackgroundInfo, isAuthenticated, hasBackgroundInfo } = useAuth();
  const history = useHistory();
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareExperience, setSoftwareExperience] = useState('');
  const [hardwareExperience, setHardwareExperience] = useState('');
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    let authResult;
    if (isLogin) {
      authResult = await signIn(email, password);
    } else {
      authResult = await signUp(email, password);
    }

    if (authResult.success) {
      if (authResult.needsBackground) {
        // Stay on this page to collect background info
      } else {
        history.push('/docs/intro'); // Redirect to docs if background not needed or already set
      }
    } else {
      setError(authResult.error || 'Authentication failed.');
    }
    setLoading(false);
  };

  const handleBackgroundSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    const result = await submitBackgroundInfo(softwareExperience, hardwareExperience);
    if (result.success) {
      history.push('/docs/intro'); // Redirect after submitting background info
    } else {
      setError(result.error || 'Failed to submit background information.');
    }
    setLoading(false);
  };

  if (isAuthenticated && hasBackgroundInfo) {
    history.push('/docs/intro'); // Already authenticated and background set, redirect
    return null;
  }

  return (
    <Layout title="Sign Up / Sign In" description="Sign up or sign in to Physical AI & Humanoid Robotics Course.">
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '2rem 0' }}>
        <div style={{ maxWidth: '500px', width: '100%', padding: '2rem', border: '1px solid #ddd', borderRadius: '8px', boxShadow: '0 4px 8px rgba(0,0,0,0.1)' }}>
          <h1>{isLogin ? 'Sign In' : 'Sign Up'}</h1>
          {error && <p style={{ color: 'red' }}>{error}</p>}

          {!isAuthenticated && (
            <form onSubmit={handleSubmit}>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="email" style={{ display: 'block', marginBottom: '0.5rem' }}>Email:</label>
                <input
                  type="email"
                  id="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  style={{ width: '100%', padding: '0.5rem', borderRadius: '4px', border: '1px solid #ccc' }}
                />
              </div>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="password" style={{ display: 'block', marginBottom: '0.5rem' }}>Password:</label>
                <input
                  type="password"
                  id="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  style={{ width: '100%', padding: '0.5rem', borderRadius: '4px', border: '1px solid #ccc' }}
                />
              </div>
              <button type="submit" disabled={loading} style={{ width: '100%', padding: '0.75rem', background: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}>
                {loading ? 'Loading...' : (isLogin ? 'Sign In' : 'Sign Up')}
              </button>
              <p style={{ textAlign: 'center', marginTop: '1rem' }}>
                {isLogin ? "Don't have an account?" : "Already have an account?"}{' '}
                <span onClick={() => setIsLogin(!isLogin)} style={{ color: '#007bff', cursor: 'pointer' }}>
                  {isLogin ? 'Sign Up' : 'Sign In'}
                </span>
              </p>
            </form>
          )}

          {isAuthenticated && !hasBackgroundInfo && (
            <form onSubmit={handleBackgroundSubmit}>
              <h2>Tell Us About Yourself</h2>
              <p>To personalize your learning experience, please tell us about your background.</p>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="softwareExperience" style={{ display: 'block', marginBottom: '0.5rem' }}>Software Development Experience:</label>
                <textarea
                  id="softwareExperience"
                  value={softwareExperience}
                  onChange={(e) => setSoftwareExperience(e.target.value)}
                  required
                  rows="4"
                  style={{ width: '100%', padding: '0.5rem', borderRadius: '4px', border: '1px solid #ccc' }}
                ></textarea>
              </div>
              <div style={{ marginBottom: '1rem' }}>
                <label htmlFor="hardwareExperience" style={{ display: 'block', marginBottom: '0.5rem' }}>Hardware/Robotics Experience:</label>
                <textarea
                  id="hardwareExperience"
                  value={hardwareExperience}
                  onChange={(e) => setHardwareExperience(e.target.value)}
                  required
                  rows="4"
                  style={{ width: '100%', padding: '0.5rem', borderRadius: '4px', border: '1px solid #ccc' }}
                ></textarea>
              </div>
              <button type="submit" disabled={loading} style={{ width: '100%', padding: '0.75rem', background: '#28a745', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}>
                {loading ? 'Submitting...' : 'Submit Background Info'}
              </button>
            </form>
          )}
        </div>
      </div>
    </Layout>
  );
};

export default SignUpForm;
