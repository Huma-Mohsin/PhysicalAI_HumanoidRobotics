/**
 * Language Context
 *
 * Manages content language (English/Urdu) based on user preference
 */

import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';

export type Language = 'en' | 'ur';

interface LanguageContextType {
  language: Language;
  setLanguage: (lang: Language) => void;
  isRTL: boolean;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

export const LanguageProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { user } = useAuth();

  // Get language from user profile or default to English
  const getUserLanguage = (): Language => {
    if (user?.hardware_profile?.language_preference) {
      return user.hardware_profile.language_preference;
    }
    return 'en';
  };

  const [language, setLanguageState] = useState<Language>(getUserLanguage());

  // Update language when user profile changes
  useEffect(() => {
    setLanguageState(getUserLanguage());
  }, [user?.hardware_profile?.language_preference]);

  // Persist language to localStorage
  const setLanguage = (lang: Language) => {
    setLanguageState(lang);
    localStorage.setItem('preferred_language', lang);

    // Update document direction for RTL
    document.documentElement.dir = lang === 'ur' ? 'rtl' : 'ltr';
    document.documentElement.lang = lang;
  };

  // Load language from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem('preferred_language') as Language;
    if (stored && ['en', 'ur'].includes(stored)) {
      setLanguageState(stored);
      document.documentElement.dir = stored === 'ur' ? 'rtl' : 'ltr';
      document.documentElement.lang = stored;
    }
  }, []);

  const isRTL = language === 'ur';

  return (
    <LanguageContext.Provider value={{ language, setLanguage, isRTL }}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = (): LanguageContextType => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within LanguageProvider');
  }
  return context;
};
