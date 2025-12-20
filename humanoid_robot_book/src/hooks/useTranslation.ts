import { useState, useEffect, useCallback } from 'react';
import { translateText, isContentInUrdu, clearTranslationCache } from '../utils/translation';

// Define the supported languages
type Language = 'en' | 'ur';

// Define the hook return type
interface UseTranslationReturn {
  currentLang: Language;
  isTranslating: boolean;
  error: string | null;
  toggleLanguage: () => void;
  translateContent: (content: string) => Promise<string>;
  isUrduMode: boolean;
  resetCache: () => void;
}

export const useTranslation = (): UseTranslationReturn => {
  // Initialize language preference from localStorage or default to English
  const [currentLang, setCurrentLang] = useState<Language>(() => {
    if (typeof window !== 'undefined') {
      const savedLang = localStorage.getItem('preferredLanguage');
      return (savedLang === 'ur' || savedLang === 'en') ? savedLang as Language : 'en';
    }
    return 'en';
  });

  const [isTranslating, setIsTranslating] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  // Update localStorage when language changes
  useEffect(() => {
    localStorage.setItem('preferredLanguage', currentLang);
  }, [currentLang]);

  // Toggle between English and Urdu
  const toggleLanguage = useCallback(() => {
    setCurrentLang(prev => prev === 'en' ? 'ur' : 'en');
  }, []);

  // Translate content with loading/error states
  const translateContent = useCallback(async (content: string): Promise<string> => {
    setIsTranslating(true);
    setError(null);

    try {
      const targetLang = currentLang;
      const result = await translateText(content, targetLang);
      return result;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed';
      setError(errorMessage);
      console.error('Translation error:', err);
      // Return original content if translation fails
      return content;
    } finally {
      setIsTranslating(false);
    }
  }, [currentLang]);

  // Check if currently in Urdu mode
  const isUrduMode = currentLang === 'ur';

  // Reset cache function
  const resetCache = useCallback(() => {
    clearTranslationCache();
  }, []);

  return {
    currentLang,
    isTranslating,
    error,
    toggleLanguage,
    translateContent,
    isUrduMode,
    resetCache
  };
};