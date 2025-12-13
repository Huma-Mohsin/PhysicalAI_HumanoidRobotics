/**
 * Root theme wrapper for Docusaurus
 * This wraps the entire application and adds the global chatbot widget
 */

import React from 'react';
import Chatbot from '../components/Chatbot/Chatbot';

export default function Root({ children }): JSX.Element {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
