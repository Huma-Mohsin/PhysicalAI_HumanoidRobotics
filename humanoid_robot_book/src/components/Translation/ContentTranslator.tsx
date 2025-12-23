import React, { useState, useEffect } from 'react';
import { useTranslationContext } from './TranslationProvider';

interface ContentTranslatorProps {
  children: string;
  className?: string;
  tag?: keyof JSX.IntrinsicElements; // Allows specifying which HTML tag to use
}

const ContentTranslator: React.FC<ContentTranslatorProps> = ({
  children,
  className = '',
  tag: Tag = 'div'
}) => {
  const { currentLang, translateContent, isTranslating, error } = useTranslationContext();
  const [translatedContent, setTranslatedContent] = useState<string>(children);

  useEffect(() => {
    const translate = async () => {
      if (currentLang === 'ur' && children.trim() !== '') {
        const result = await translateContent(children);
        setTranslatedContent(result);
      } else {
        // When switching back to English, use original content
        setTranslatedContent(children);
      }
    };

    translate();
  }, [children, currentLang, translateContent]);

  // Determine if content is currently in Urdu mode
  const isInUrduMode = currentLang === 'ur' && children.trim() !== '';
  const finalContent = isInUrduMode ? translatedContent : children;

  // Apply RTL styling when in Urdu mode
  const contentClassName = `${className} ${isInUrduMode ? 'urduText' : ''}`.trim();

  return (
    <Tag
      className={contentClassName}
      dir={isInUrduMode ? 'rtl' : 'ltr'}
    >
      {finalContent}
    </Tag>
  );
};

export default ContentTranslator;