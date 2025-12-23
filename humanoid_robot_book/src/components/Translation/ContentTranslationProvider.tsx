import React, { createContext, useContext, ReactNode } from 'react';
import { useTranslationContext } from './TranslationProvider';

interface ContentTranslationContextType {
  currentLang: 'en' | 'ur';
}

const ContentTranslationContext = createContext<ContentTranslationContextType | undefined>(undefined);

export const ContentTranslationProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { currentLang } = useTranslationContext();

  return (
    <ContentTranslationContext.Provider value={{ currentLang }}>
      {children}
    </ContentTranslationContext.Provider>
  );
};

export const useContentTranslation = (): ContentTranslationContextType => {
  const context = useContext(ContentTranslationContext);
  if (context === undefined) {
    throw new Error('useContentTranslation must be used within a ContentTranslationProvider');
  }
  return context;
};