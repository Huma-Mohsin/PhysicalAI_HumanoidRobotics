/**
 * Sign Up Page
 *
 * User registration with hardware profile setup
 */

import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function SignUp() {
  const { signUp, isAuthenticated } = useAuth();
  const history = useHistory();

  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    os_type: '',
    gpu_model: '',
    environment_preference: 'cloud' as 'workstation' | 'cloud' | 'mac',
    language_preference: 'en' as 'en' | 'ur',
  });

  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  // Redirect if already authenticated
  React.useEffect(() => {
    if (isAuthenticated) {
      history.push('/');
    }
  }, [isAuthenticated, history]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value,
    });
    setError(null);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validation
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setIsLoading(true);

    const result = await signUp({
      email: formData.email,
      password: formData.password,
      os_type: formData.os_type || undefined,
      gpu_model: formData.gpu_model || undefined,
      environment_preference: formData.environment_preference,
      language_preference: formData.language_preference,
    });

    setIsLoading(false);

    if (result.success) {
      history.push('/');
    } else {
      setError(result.error || 'Sign up failed');
    }
  };

  return (
    <Layout title="Sign Up" description="Create your Physical AI account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1>Create Account</h1>
          <p className={styles.subtitle}>Join the Physical AI learning platform</p>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {/* Email */}
            <div className={styles.formGroup}>
              <label htmlFor="email">Email Address *</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                required
                placeholder="student@example.com"
              />
            </div>

            {/* Password */}
            <div className={styles.formGroup}>
              <label htmlFor="password">Password *</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                required
                minLength={8}
                placeholder="Min. 8 characters"
              />
            </div>

            {/* Confirm Password */}
            <div className={styles.formGroup}>
              <label htmlFor="confirmPassword">Confirm Password *</label>
              <input
                type="password"
                id="confirmPassword"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleChange}
                required
                placeholder="Re-enter password"
              />
            </div>

            <div className={styles.divider}>
              <span>Hardware Profile (Optional)</span>
            </div>

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
            </div>

            {/* Error Message */}
            {error && (
              <div className={styles.errorMessage}>
                {error}
              </div>
            )}

            {/* Submit Button */}
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? 'Creating Account...' : 'Sign Up'}
            </button>
          </form>

          <p className={styles.footerLink}>
            Already have an account? <a href="/signin">Sign In</a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
