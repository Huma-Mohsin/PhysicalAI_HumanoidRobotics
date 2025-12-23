import React, { useState } from 'react';
import { useTranslationContext } from './TranslationProvider';
import styles from './TranslationToggle.module.css';

const TranslationToggle: React.FC = () => {
  const { currentLang, isTranslating, toggleLanguage, error } = useTranslationContext();
  const [showError, setShowError] = useState(false);

  // Show error briefly if one occurs
  React.useEffect(() => {
    if (error) {
      setShowError(true);
      const timer = setTimeout(() => setShowError(false), 5000); // Hide error after 5 seconds
      return () => clearTimeout(timer);
    }
  }, [error]);

  const handleClick = () => {
    toggleLanguage();

    // Navigate to Urdu or English version using Docusaurus i18n routing
    const currentPath = window.location.pathname;

    if (currentLang === 'en') {
      // Switch to Urdu
      const urduPath = currentPath.startsWith('/ur/')
        ? currentPath
        : `/ur${currentPath}`;
      window.location.href = urduPath;
    } else {
      // Switch to English
      const englishPath = currentPath.replace(/^\/ur/, '');
      window.location.href = englishPath || '/';
    }
  };

  return (
    <div className={styles.translationContainer}>
      <button
        onClick={handleClick}
        disabled={isTranslating}
        className={`${styles.translationButton} ${currentLang === 'ur' ? styles.urduActive : styles.englishActive}`}
        aria-label={currentLang === 'ur' ? 'Switch to English' : 'Translate to Urdu'}
        title={currentLang === 'ur' ? 'Switch to English' : 'Translate to Urdu'}
      >
        {isTranslating ? (
          <span className={styles.loadingSpinner}>
            <span className={styles.spinner}></span> Translating...
          </span>
        ) : (
          <span className={styles.buttonText}>
            {currentLang === 'ur' ? 'English' : 'اردو'}
          </span>
        )}
      </button>

      {showError && (
        <div className={styles.errorMessage}>
          Translation error: {error}
        </div>
      )}
    </div>
  );
};

export default TranslationToggle;