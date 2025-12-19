/**
 * Content Variant Component
 * Feature 005: Interactive Personalization System
 * Feature 008: Prioritize auth hardware data over localStorage
 *
 * MDX wrapper component that shows/hides hardware-specific content based on:
 * 1. User's hardware profile (from auth first, then localStorage)
 * 2. Personalization toggle state
 *
 * Usage in MDX:
 * <ContentVariant hardwareType="gpu_workstation">
 *   ## GPU-specific content here
 * </ContentVariant>
 */

import React, { ReactNode } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import { HardwareType, HARDWARE_LABELS } from '../../types/hardware';
import styles from './ContentVariant.module.css';

interface ContentVariantProps {
  hardwareType: HardwareType;
  children: ReactNode;
}

export default function ContentVariant({ hardwareType, children }: ContentVariantProps) {
  const { user } = useAuth();
  const { profile, isPersonalized } = useHardwareProfile();

  // Feature 008: Prioritize auth hardware data, fallback to localStorage profile
  const userHardwareType = user?.hardwareProfile?.hardwareType || profile?.type;

  // Determine if this variant matches user's hardware
  const isUserHardware = userHardwareType === hardwareType;

  // Determine display mode
  let variantClass = styles.variant;

  if (isPersonalized) {
    if (isUserHardware) {
      // User's hardware - highlight
      variantClass += ` ${styles.variantActive}`;
    } else {
      // Other hardware - dim
      variantClass += ` ${styles.variantDimmed}`;
    }
  }

  return (
    <div className={variantClass}>
      <div className={styles.variantHeader}>
        <span className={styles.variantLabel}>
          {isUserHardware && isPersonalized ? '✓ ' : ''}
          For: {HARDWARE_LABELS[hardwareType]}
        </span>
        {isUserHardware && isPersonalized && (
          <span className={styles.variantBadge}>Your Setup</span>
        )}
      </div>
      <div className={styles.variantContent}>
        {children}
      </div>
    </div>
  );
}
