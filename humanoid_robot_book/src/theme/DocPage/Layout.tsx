import React from 'react';
import { useDocsPage } from '@docusaurus/theme-common/internal';
import { useTranslationContext } from '../../components/Translation/TranslationProvider';
import TranslationInterceptor from '../../components/Translation/TranslationInterceptor';

interface Props {
  children: React.ReactNode;
}

const Layout: React.FC<Props> = ({ children }) => {
  const { currentLang } = useTranslationContext();

  return (
    <div dir={currentLang === 'ur' ? 'rtl' : 'ltr'} className={currentLang === 'ur' ? 'urdu-content' : ''}>
      <TranslationInterceptor>
        {children}
      </TranslationInterceptor>
    </div>
  );
};

export default Layout;