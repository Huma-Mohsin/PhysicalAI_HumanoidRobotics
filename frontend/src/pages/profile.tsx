/**
 * Hardware Profile Settings Page
 *
 * Update hardware preferences and view account info
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function Profile() {
  const { user, isAuthenticated, updateHardwareProfile, signOut } = useAuth();
  const history = useHistory();

  const [formData, setFormData] = useState({
    os_type: '',
    gpu_model: '',
    environment_preference: 'cloud' as 'workstation' | 'cloud' | 'mac',
    language_preference: 'en' as 'en' | 'ur',
  });

  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  // Redirect if not authenticated
  useEffect(() => {
    if (!isAuthenticated) {
      history.push('/signin');
    }
  }, [isAuthenticated, history]);

  // Load current profile data
  useEffect(() => {
    if (user?.hardware_profile) {
      setFormData({
        os_type: user.hardware_profile.os_type || '',
        gpu_model: user.hardware_profile.gpu_model || '',
        environment_preference: user.hardware_profile.environment_preference,
        language_preference: user.hardware_profile.language_preference,
      });
    }
  }, [user]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value,
    });
    setMessage(null);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setMessage(null);
    setIsLoading(true);

    const result = await updateHardwareProfile(formData);

    setIsLoading(false);

    if (result.success) {
      setMessage({ type: 'success', text: 'Profile updated successfully!' });
    } else {
      setMessage({ type: 'error', text: result.error || 'Update failed' });
    }
  };

  const handleSignOut = () => {
    signOut();
    history.push('/');
  };

  if (!user) {
    return null; // Will redirect
  }

  return (
    <Layout title="Profile" description="Manage your hardware profile">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Hardware Profile</h1>
          <p className={styles.subtitle}>Customize your learning experience</p>

          {/* Account Info */}
          <div style={{ marginBottom: '24px', padding: '16px', background: '#f8f9fa', borderRadius: '8px' }}>
            <div><strong>Email:</strong> {user.email}</div>
            <div style={{ marginTop: '8px', fontSize: '13px', color: '#6c757d' }}>
              Account created: {new Date(user.created_at).toLocaleDateString()}
            </div>
            {user.hardware_profile?.is_workstation_capable && (
              <div style={{ marginTop: '8px', padding: '8px', background: '#d4edda', color: '#155724', borderRadius: '4px', fontSize: '14px' }}>
                ✅ Workstation Mode Available (GPU: {user.hardware_profile.gpu_model})
              </div>
            )}
          </div>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {/* OS Type */}
            <div className={styles.formGroup}>
              <label htmlFor="os_type">Operating System</label>
              <input
                type="text"
                id="os_type"
                name="os_type"
                value={formData.os_type}
                onChange={handleChange}
                placeholder="e.g., Ubuntu 22.04, Windows 11, macOS"
              />
            </div>

            {/* GPU Model */}
            <div className={styles.formGroup}>
              <label htmlFor="gpu_model">GPU Model</label>
              <input
                type="text"
                id="gpu_model"
                name="gpu_model"
                value={formData.gpu_model}
                onChange={handleChange}
                placeholder="e.g., RTX 4070 Ti, RTX 3060, M1 Pro"
              />
              <small>RTX 4070 Ti+ recommended for Isaac Sim workstation mode</small>
            </div>

            {/* Environment Preference */}
            <div className={styles.formGroup}>
              <label htmlFor="environment_preference">Preferred Environment</label>
              <select
                id="environment_preference"
                name="environment_preference"
                value={formData.environment_preference}
                onChange={handleChange}
              >
                <option value="cloud">Cloud (Default)</option>
                <option value="workstation">Workstation (RTX 4070 Ti+)</option>
                <option value="mac">Mac</option>
              </select>
              <small>
                {formData.environment_preference === 'workstation'
                  ? 'Shows NVIDIA Isaac Sim local installation guides'
                  : formData.environment_preference === 'mac'
                  ? 'Shows Mac-compatible alternatives (no local Isaac Sim)'
                  : 'Shows Omniverse Cloud-based workflows'}
              </small>
            </div>

            {/* Language Preference */}
            <div className={styles.formGroup}>
              <label htmlFor="language_preference">Content Language</label>
              <select
                id="language_preference"
                name="language_preference"
                value={formData.language_preference}
                onChange={handleChange}
              >
                <option value="en">English</option>
                <option value="ur">اردو (Urdu)</option>
              </select>
              <small>Technical terms preserved in English for accuracy</small>
            </div>

            {/* Message */}
            {message && (
              <div className={message.type === 'success' ? styles.successMessage : styles.errorMessage}>
                {message.text}
              </div>
            )}

            {/* Buttons */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? 'Updating...' : 'Save Changes'}
            </button>

            <button
              type="button"
              onClick={handleSignOut}
              style={{
                background: 'transparent',
                border: '1px solid #dee2e6',
                color: '#6c757d',
                padding: '12px',
                borderRadius: '8px',
                cursor: 'pointer',
                fontWeight: 600,
              }}
            >
              Sign Out
            </button>
          </form>
        </div>
      </div>
    </Layout>
  );
}
