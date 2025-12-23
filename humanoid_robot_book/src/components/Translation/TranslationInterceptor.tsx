import React, { useEffect, useRef, useState } from 'react';
import { useTranslationContext } from './TranslationProvider';
import { translateDocument } from '../../utils/translation';

interface TranslationInterceptorProps {
  children: React.ReactNode;
}

const TranslationInterceptor: React.FC<TranslationInterceptorProps> = ({ children }) => {
  const { currentLang } = useTranslationContext();
  const containerRef = useRef<HTMLDivElement>(null);
  const [hasTranslated, setHasTranslated] = useState(false);

  useEffect(() => {
    if (!containerRef.current) return;

    const container = containerRef.current;

    const translateContent = async () => {
      // Wait a bit for content to render
      await new Promise(resolve => setTimeout(resolve, 100));

      if (currentLang === 'ur') {
        // Store original content if not already stored
        if (!container.dataset.originalContent) {
          container.dataset.originalContent = container.innerHTML;
        }

        const contentToTranslate = container.innerHTML;
        const translatedContent = await translateDocument(contentToTranslate, 'ur');

        // Temporarily disable React's ability to update this element
        container.innerHTML = translatedContent;
        container.dir = 'rtl';
        setHasTranslated(true);
      } else {
        // Restore original content
        if (container.dataset.originalContent) {
          container.innerHTML = container.dataset.originalContent;
        }
        container.dir = 'ltr';
        setHasTranslated(true);
      }
    };

    translateContent();
  }, [currentLang]);

  // Only render children initially, then let the effect handle the translation
  return (
    <div
      ref={containerRef}
      dir={currentLang === 'ur' ? 'rtl' : 'ltr'}
      className={currentLang === 'ur' ? 'urdu-content' : ''}
    >
      {children}
    </div>
  );
};

export default TranslationInterceptor;