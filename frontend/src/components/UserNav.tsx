/**
 * User Navigation Component
 *
 * Shows user menu in navbar when authenticated
 */

import React from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './UserNav.module.css';

export const UserNav: React.FC = () => {
  const { user, isAuthenticated, signOut } = useAuth();

  if (!isAuthenticated || !user) {
    return (
      <div className={styles.authLinks}>
        <a href="/signin" className={styles.signInLink}>Sign In</a>
        <a href="/signup" className={styles.signUpButton}>Sign Up</a>
      </div>
    );
  }

  return (
    <div className={styles.userMenu}>
      <div className={styles.userInfo}>
        <span className={styles.userEmail}>{user.email}</span>
        {user.hardware_profile && (
          <span className={styles.userMode}>
            {user.hardware_profile.environment_preference === 'workstation' && '💻 Workstation'}
            {user.hardware_profile.environment_preference === 'cloud' && '☁️ Cloud'}
            {user.hardware_profile.environment_preference === 'mac' && '🍎 Mac'}
          </span>
        )}
      </div>
      <div className={styles.userDropdown}>
        <a href="/profile" className={styles.dropdownLink}>⚙️ Profile</a>
        <button onClick={signOut} className={styles.dropdownLink}>
          🚪 Sign Out
        </button>
      </div>
    </div>
  );
};
