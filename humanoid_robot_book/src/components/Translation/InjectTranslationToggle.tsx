/**
 * Component that injects the translation toggle at the top of documentation content
 */
import React from 'react';
import TranslationToggle from './TranslationToggle';

const InjectTranslationToggle: React.FC = () => {
  return (
    <div
      style={{
        display: 'flex',
        justifyContent: 'flex-end',
        marginBottom: '1rem',
        marginTop: '1rem'
      }}
    >
      <TranslationToggle />
    </div>
  );
};

export default InjectTranslationToggle;