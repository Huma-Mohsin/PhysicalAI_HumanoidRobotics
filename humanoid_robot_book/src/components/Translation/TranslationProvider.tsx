import React, { createContext, useContext, ReactNode } from 'react';
import { useTranslation } from '../../hooks/useTranslation';

// Define the context type
interface TranslationContextType {
  currentLang: 'en' | 'ur';
  isTranslating: boolean;
  error: string | null;
  toggleLanguage: () => void;
  translateContent: (content: string) => Promise<string>;
  isUrduMode: boolean;
  resetCache: () => void;
}

// Create the context
const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Define props type
interface TranslationProviderProps {
  children: ReactNode;
}

// Translation Provider Component
export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  const translationHook = useTranslation();

  return (
    <TranslationContext.Provider value={translationHook}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslationContext = (): TranslationContextType => {
  const context = useContext(TranslationContext);
  if (context === undefined) {
    throw new Error('useTranslationContext must be used within a TranslationProvider');
  }
  return context;
};