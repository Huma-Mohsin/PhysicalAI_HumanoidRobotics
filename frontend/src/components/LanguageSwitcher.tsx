/**
 * Language Switcher Component
 *
 * Toggle between English and Urdu
 */

import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import { useAuth } from '../hooks/useAuth';
import styles from './LanguageSwitcher.module.css';

export const LanguageSwitcher: React.FC = () => {
  const { language, setLanguage } = useLanguage();
  const { isAuthenticated, updateHardwareProfile } = useAuth();

  const handleLanguageChange = async (newLang: 'en' | 'ur') => {
    setLanguage(newLang);

    // Update user profile if authenticated
    if (isAuthenticated) {
      await updateHardwareProfile({
        language_preference: newLang,
      });
    }
  };

  return (
    <div className={styles.switcherContainer}>
      <button
        className={`${styles.langButton} ${language === 'en' ? styles.active : ''}`}
        onClick={() => handleLanguageChange('en')}
        aria-label="Switch to English"
      >
        🇬🇧 English
      </button>

      <button
        className={`${styles.langButton} ${language === 'ur' ? styles.active : ''}`}
        onClick={() => handleLanguageChange('ur')}
        aria-label="اردو میں تبدیل کریں"
      >
        🇵🇰 اردو
      </button>

      {!isAuthenticated && language === 'ur' && (
        <div className={styles.authPrompt}>
          <a href="/signup">سائن اپ کریں</a> اپنی زبان کی ترجیحات محفوظ کرنے کے لیے
        </div>
      )}
    </div>
  );
};
