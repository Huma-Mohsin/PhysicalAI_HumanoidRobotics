/**
 * Environment Switcher Component
 *
 * Allows users to manually toggle between environment modes
 */

import React from 'react';
import { useEnvironment, EnvironmentMode } from '../contexts/EnvironmentContext';
import { useAuth } from '../hooks/useAuth';
import styles from './EnvironmentSwitcher.module.css';

export const EnvironmentSwitcher: React.FC = () => {
  const { mode, setMode, isWorkstationCapable, autoDetectedMode } = useEnvironment();
  const { isAuthenticated } = useAuth();

  const handleModeChange = (newMode: EnvironmentMode) => {
    setMode(newMode);
  };

  return (
    <div className={styles.switcherContainer}>
      <div className={styles.switcherHeader}>
        <span className={styles.switcherLabel}>Environment Mode</span>
        {mode !== autoDetectedMode && (
          <button
            onClick={() => setMode(autoDetectedMode)}
            className={styles.resetButton}
          >
            Reset to {autoDetectedMode}
          </button>
        )}
      </div>

      <div className={styles.modeButtons}>
        <button
          className={`${styles.modeButton} ${mode === 'workstation' ? styles.active : ''} ${
            !isWorkstationCapable ? styles.disabled : ''
          }`}
          onClick={() => handleModeChange('workstation')}
          disabled={!isWorkstationCapable}
          title={
            !isWorkstationCapable
              ? 'Requires RTX 4070 Ti+ GPU. Update your hardware profile to enable.'
              : 'Local Isaac Sim installation'
          }
        >
          <span className={styles.modeIcon}>💻</span>
          <div className={styles.modeInfo}>
            <span className={styles.modeName}>Workstation</span>
            <span className={styles.modeDesc}>RTX 4070 Ti+</span>
          </div>
          {!isWorkstationCapable && <span className={styles.lockIcon}>🔒</span>}
        </button>

        <button
          className={`${styles.modeButton} ${mode === 'cloud' ? styles.active : ''}`}
          onClick={() => handleModeChange('cloud')}
          title="Omniverse Cloud workflows"
        >
          <span className={styles.modeIcon}>☁️</span>
          <div className={styles.modeInfo}>
            <span className={styles.modeName}>Cloud</span>
            <span className={styles.modeDesc}>Omniverse Cloud</span>
          </div>
        </button>

        <button
          className={`${styles.modeButton} ${mode === 'mac' ? styles.active : ''}`}
          onClick={() => handleModeChange('mac')}
          title="Mac-compatible alternatives"
        >
          <span className={styles.modeIcon}>🍎</span>
          <div className={styles.modeInfo}>
            <span className={styles.modeName}>Mac</span>
            <span className={styles.modeDesc}>macOS</span>
          </div>
        </button>
      </div>

      {!isAuthenticated && (
        <div className={styles.authPrompt}>
          <a href="/signup">Sign up</a> to save your hardware preferences
        </div>
      )}
    </div>
  );
};
