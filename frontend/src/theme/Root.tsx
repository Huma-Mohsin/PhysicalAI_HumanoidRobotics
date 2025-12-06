/**
 * Docusaurus Root Theme Component
 *
 * This wraps the entire Docusaurus site and provides global context providers
 */

import React, { useState, useEffect } from 'react';
import { ChatInterface } from '../components/ChatInterface';
import { EnvironmentProvider } from '../contexts/EnvironmentContext';
import { LanguageProvider } from '../contexts/LanguageContext';
import '../styles/rtl.css';

export default function Root({ children }) {
  const [selectedText, setSelectedText] = useState<string | undefined>();

  // Listen for text selection events
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
      } else if (!text) {
        setSelectedText(undefined);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  return (
    <LanguageProvider>
      <EnvironmentProvider>
        {children}
        <ChatInterface selectedText={selectedText} />
      </EnvironmentProvider>
    </LanguageProvider>
  );
}
