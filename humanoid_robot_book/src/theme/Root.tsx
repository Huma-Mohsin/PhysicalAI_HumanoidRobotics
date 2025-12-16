/**
 * Root theme wrapper for Docusaurus
 * This wraps the entire application and adds the global chatbot widget
 */

import React, { useState } from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import Chatbot from '../components/Chatbot/Chatbot';
import TextSelection from '../components/TextSelection/TextSelection';

export default function Root({ children }): JSX.Element {
  const [selectedText, setSelectedText] = useState('');

  return (
    <AuthProvider>
      <>
        {children}
        <TextSelection onAskAboutSelection={(text) => setSelectedText(text)} />
        <Chatbot
          selectedText={selectedText}
          onSelectedTextUsed={() => setSelectedText('')}
        />
      </>
    </AuthProvider>
  );
}
