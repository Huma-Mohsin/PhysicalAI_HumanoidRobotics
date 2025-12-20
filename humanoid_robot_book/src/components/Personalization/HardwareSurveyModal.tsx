/**
 * Hardware Survey Modal
 * Feature 005: Interactive Personalization System
 * Feature 008: Skip modal if user has hardware data from signup
 *
 * Shows on first visit to collect user's hardware profile.
 * Saves to localStorage + syncs to backend if logged in.
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import { HardwareType, HARDWARE_LABELS, HARDWARE_DESCRIPTIONS } from '../../types/hardware';
import styles from './HardwareSurveyModal.module.css';

export default function HardwareSurveyModal() {
  const { user } = useAuth();
  const { profile, setProfile } = useHardwareProfile();
  const [showModal, setShowModal] = useState(false);
  const [selectedType, setSelectedType] = useState<HardwareType>('cloud_mac');

  // Show modal 1 second after page load (only if no profile exists AND user has no hardware from signup)
  useEffect(() => {
    const timer = setTimeout(() => {
      // Feature 008: Skip modal if user has hardware data from signup OR login
      // Check BOTH hardwareProfile.hardwareType (signup) and hardwareType (login)
      const hasAuthHardware = user?.hardwareProfile?.hardwareType || user?.hardwareType;

      if (!profile && !hasAuthHardware) {
        console.log('[HardwareSurveyModal] No profile found, showing modal');
        setShowModal(true);
      } else {
        console.log('[HardwareSurveyModal] Profile or auth hardware exists, skipping modal');
      }
    }, 1000);

    return () => clearTimeout(timer);
  }, [profile, user]);

  const handleContinue = async () => {
    console.log('[HardwareSurveyModal] User selected:', selectedType);

    await setProfile({
      type: selectedType,
      selectedAt: new Date(),
    });

    setShowModal(false);
  };

  const handleSkip = async () => {
    console.log('[HardwareSurveyModal] User skipped, setting default cloud_mac');

    await setProfile({
      type: 'cloud_mac',
      selectedAt: new Date(),
    });

    setShowModal(false);
  };

  if (!showModal) {
    return null;
  }

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modalContent}>
        <h2 className={styles.modalTitle}>Welcome to Physical AI & Humanoid Robotics!</h2>
        <p className={styles.modalDescription}>
          To personalize your learning experience, please select your hardware setup:
        </p>

        <div className={styles.optionsContainer}>
          {(['gpu_workstation', 'edge_device', 'cloud_mac'] as HardwareType[]).map((type) => (
            <label
              key={type}
              className={`${styles.optionCard} ${selectedType === type ? styles.selected : ''}`}
            >
              <input
                type="radio"
                name="hardwareType"
                value={type}
                checked={selectedType === type}
                onChange={() => setSelectedType(type)}
                className={styles.radioInput}
              />
              <div className={styles.optionContent}>
                <h3 className={styles.optionLabel}>{HARDWARE_LABELS[type]}</h3>
                <p className={styles.optionDescription}>{HARDWARE_DESCRIPTIONS[type]}</p>
              </div>
            </label>
          ))}
        </div>

        <div className={styles.modalActions}>
          <button
            onClick={handleSkip}
            className={`${styles.button} ${styles.buttonSecondary}`}
          >
            Skip for now
          </button>
          <button
            onClick={handleContinue}
            className={`${styles.button} ${styles.buttonPrimary}`}
          >
            Continue
          </button>
        </div>

        <p className={styles.modalFooter}>
          You can update your hardware profile anytime in Settings.
        </p>
      </div>
    </div>
  );
}
