/**
 * Environment Badge Component
 *
 * Shows which environment mode(s) content applies to
 */

import React from 'react';
import { EnvironmentMode } from '../contexts/EnvironmentContext';
import styles from './EnvironmentBadge.module.css';

interface EnvironmentBadgeProps {
  modes: EnvironmentMode[];
}

export const EnvironmentBadge: React.FC<EnvironmentBadgeProps> = ({ modes }) => {
  const getBadgeConfig = (mode: EnvironmentMode) => {
    switch (mode) {
      case 'workstation':
        return { label: '💻 Workstation', className: styles.workstation };
      case 'cloud':
        return { label: '☁️ Cloud', className: styles.cloud };
      case 'mac':
        return { label: '🍎 Mac', className: styles.mac };
    }
  };

  return (
    <div className={styles.badgeContainer}>
      {modes.map((mode) => {
        const config = getBadgeConfig(mode);
        return (
          <span key={mode} className={`${styles.badge} ${config.className}`}>
            {config.label}
          </span>
        );
      })}
    </div>
  );
};
