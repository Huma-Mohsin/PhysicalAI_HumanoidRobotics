/**
 * Root theme wrapper for Docusaurus
 * This wraps the entire application and adds the global chatbot widget
 */

import React, { useState } from 'react';
import Chatbot from '../components/Chatbot/Chatbot';
import TextSelection from '../components/TextSelection/TextSelection';

export default function Root({ children }): JSX.Element {
  const [selectedText, setSelectedText] = useState('');

  return (
    <>
      {children}
      <TextSelection onAskAboutSelection={(text) => setSelectedText(text)} />
      <Chatbot
        selectedText={selectedText}
        onSelectedTextUsed={() => setSelectedText('')}
      />
    </>
  );
}
