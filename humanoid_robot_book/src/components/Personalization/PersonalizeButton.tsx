/**
 * Personalize Button Component
 * Feature 005: Interactive Personalization System
 *
 * Toggles personalization mode to highlight user's hardware-specific content.
 * Shows at the start of chapters that have hardware-specific variants.
 */

import React from 'react';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import { HARDWARE_LABELS } from '../../types/hardware';
import styles from './PersonalizeButton.module.css';

export default function PersonalizeButton() {
  const { profile, isPersonalized, togglePersonalization } = useHardwareProfile();

  // Don't show button if no profile selected
  if (!profile) {
    return (
      <div className={styles.noProfileMessage}>
        <p>
          💡 <strong>Tip:</strong> Select your hardware profile to see personalized content for your setup.
        </p>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.profileInfo}>
        <span className={styles.profileLabel}>Your hardware:</span>
        <span className={styles.profileValue}>{HARDWARE_LABELS[profile.type]}</span>
      </div>

      <button
        onClick={togglePersonalization}
        className={`${styles.button} ${isPersonalized ? styles.buttonActive : ''}`}
      >
        {isPersonalized ? (
          <>
            <span className={styles.icon}>✓</span>
            <span>Personalized View</span>
          </>
        ) : (
          <>
            <span className={styles.icon}>◇</span>
            <span>Show All Options</span>
          </>
        )}
      </button>

      {isPersonalized && (
        <p className={styles.hint}>
          Content is now focused on your <strong>{HARDWARE_LABELS[profile.type]}</strong> setup.
          Click again to see all hardware options.
        </p>
      )}
    </div>
  );
}
