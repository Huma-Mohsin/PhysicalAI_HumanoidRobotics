/**
 * Environment Content Component
 *
 * Conditionally renders content based on current environment mode
 */

import React from 'react';
import { useEnvironment, EnvironmentMode } from '../contexts/EnvironmentContext';
import { EnvironmentBadge } from './EnvironmentBadge';
import styles from './EnvironmentContent.module.css';

interface EnvironmentContentProps {
  modes: EnvironmentMode[];
  children: React.ReactNode;
  showBadge?: boolean;
}

export const EnvironmentContent: React.FC<EnvironmentContentProps> = ({
  modes,
  children,
  showBadge = true,
}) => {
  const { mode } = useEnvironment();

  // Show content if current mode is in the allowed modes
  const shouldShow = modes.includes(mode);

  if (!shouldShow) {
    return null;
  }

  return (
    <div className={styles.environmentContent}>
      {showBadge && <EnvironmentBadge modes={modes} />}
      {children}
    </div>
  );
};

/**
 * Specialized components for common use cases
 */

export const WorkstationOnly: React.FC<{ children: React.ReactNode; showBadge?: boolean }> = ({
  children,
  showBadge = true,
}) => (
  <EnvironmentContent modes={['workstation']} showBadge={showBadge}>
    {children}
  </EnvironmentContent>
);

export const CloudOnly: React.FC<{ children: React.ReactNode; showBadge?: boolean }> = ({
  children,
  showBadge = true,
}) => (
  <EnvironmentContent modes={['cloud']} showBadge={showBadge}>
    {children}
  </EnvironmentContent>
);

export const MacOnly: React.FC<{ children: React.ReactNode; showBadge?: boolean }> = ({
  children,
  showBadge = true,
}) => (
  <EnvironmentContent modes={['mac']} showBadge={showBadge}>
    {children}
  </EnvironmentContent>
);

export const CloudOrMac: React.FC<{ children: React.ReactNode; showBadge?: boolean }> = ({
  children,
  showBadge = true,
}) => (
  <EnvironmentContent modes={['cloud', 'mac']} showBadge={showBadge}>
    {children}
  </EnvironmentContent>
);
