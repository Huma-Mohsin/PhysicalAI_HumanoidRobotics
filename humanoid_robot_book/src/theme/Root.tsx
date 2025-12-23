/**
 * Root theme wrapper for Docusaurus
 * This wraps the entire application and adds the global chatbot widget and translation functionality
 *
 * Provider hierarchy:
 * - AuthProvider (Feature 007 - Better-Auth)
 * - HardwareProfileProvider (Feature 005 - Personalization)
 * - TranslationProvider (Feature 003 - Localization)
 */

import React, { useState } from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { HardwareProfileProvider } from '../contexts/HardwareProfileContext';
import { TranslationProvider } from '../components/Translation/TranslationProvider';
import HardwareSurveyModal from '../components/Personalization/HardwareSurveyModal';
import Chatbot from '../components/Chatbot/Chatbot';
import TextSelection from '../components/TextSelection/TextSelection';

export default function Root({ children }): JSX.Element {
  const [selectedText, setSelectedText] = useState('');

  return (
    <AuthProvider>
      <HardwareProfileProvider>
        <HardwareSurveyModal />
        <TranslationProvider>
          <>
            {children}
            <TextSelection onAskAboutSelection={(text) => setSelectedText(text)} />
            <Chatbot
              selectedText={selectedText}
              onSelectedTextUsed={() => setSelectedText('')}
            />
          </>
        </TranslationProvider>
      </HardwareProfileProvider>
    </AuthProvider>
  );
}
