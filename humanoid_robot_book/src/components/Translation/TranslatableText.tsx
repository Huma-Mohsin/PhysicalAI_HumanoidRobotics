import React, { useState, useEffect } from 'react';
import { useTranslationContext } from './TranslationProvider';

interface TranslatableTextProps {
  children: string;
  className?: string;
  tag?: keyof JSX.IntrinsicElements;
}

const TranslatableText: React.FC<TranslatableTextProps> = ({
  children,
  className = '',
  tag: Tag = 'span'
}) => {
  const { currentLang, translateContent, isTranslating } = useTranslationContext();
  const [translatedContent, setTranslatedContent] = useState<string>(children);

  useEffect(() => {
    const translate = async () => {
      if (currentLang === 'ur' && children.trim() !== '') {
        const result = await translateContent(children);
        setTranslatedContent(result);
      } else {
        setTranslatedContent(children);
      }
    };

    translate();
  }, [children, currentLang, translateContent]);

  const isInUrduMode = currentLang === 'ur' && children.trim() !== '';
  const finalContent = isInUrduMode ? translatedContent : children;

  return (
    <Tag
      className={`${className} ${isInUrduMode ? 'urduText' : ''}`.trim()}
      dir={isInUrduMode ? 'rtl' : 'ltr'}
    >
      {finalContent}
    </Tag>
  );
};

export default TranslatableText;