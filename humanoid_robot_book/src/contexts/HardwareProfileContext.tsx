/**
 * Hardware Profile Context
 * Feature 005: Interactive Personalization System
 *
 * Provides global state for user hardware profile with dual storage:
 * - localStorage for immediate persistence (anonymous users)
 * - Better-Auth backend sync (logged-in users from Feature 007)
 */

import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import { useAuth } from './AuthContext';
import { HardwareProfile, HardwareProfileContextValue } from '../types/hardware';

const HardwareProfileContext = createContext<HardwareProfileContextValue | null>(null);

const STORAGE_KEY = 'hardwareProfile';

interface HardwareProfileProviderProps {
  children: ReactNode;
}

export function HardwareProfileProvider({ children }: HardwareProfileProviderProps) {
  const { user } = useAuth(); // Use existing auth from Feature 007
  const [profile, setProfileState] = useState<HardwareProfile | null>(null);
  const [isPersonalized, setIsPersonalized] = useState(false);

  // Load from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored) {
      try {
        const parsed = JSON.parse(stored);
        // Convert selectedAt string back to Date
        if (parsed.selectedAt) {
          parsed.selectedAt = new Date(parsed.selectedAt);
        }
        setProfileState(parsed);
      } catch (e) {
        console.error('[HardwareProfile] Failed to parse from localStorage:', e);
        localStorage.removeItem(STORAGE_KEY); // Clear corrupted data
      }
    }
  }, []);

  // Auto-enable personalization when user logs in with hardware profile
  useEffect(() => {
    if (user?.hardwareProfile?.hardwareType || user?.hardwareType) {
      console.log('[HardwareProfile] User logged in with hardware profile, enabling personalization');
      setIsPersonalized(true);
    } else if (!user) {
      // Reset personalization when user logs out
      console.log('[HardwareProfile] User logged out, disabling personalization');
      setIsPersonalized(false);
    }
  }, [user]);

  const setProfile = async (newProfile: HardwareProfile) => {
    console.log('[HardwareProfile] Setting profile:', newProfile);

    // 1. Save to state
    setProfileState(newProfile);

    // 2. Save to localStorage
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(newProfile));
      console.log('[HardwareProfile] Saved to localStorage');
    } catch (e) {
      console.error('[HardwareProfile] Failed to save to localStorage:', e);
    }

    // 3. If logged in, sync to backend (Feature 007 integration)
    if (user?.id) {
      try {
        console.log('[HardwareProfile] Syncing to backend for user:', user.id);

        const response = await fetch('/api/profile/update', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include', // Send session cookie
          body: JSON.stringify({
            userId: user.id,
            hardwareType: newProfile.type,
            hardwareDetails: {
              gpuModel: newProfile.gpuModel,
              cpuModel: newProfile.cpuModel,
              ramSize: newProfile.ramSize,
              osType: newProfile.osType,
            },
          }),
        });

        if (!response.ok) {
          console.warn('[HardwareProfile] Backend sync failed:', response.status);
        } else {
          console.log('[HardwareProfile] Successfully synced to backend');
        }
      } catch (error) {
        console.error('[HardwareProfile] Failed to sync to backend:', error);
        // Non-critical: localStorage save already succeeded
      }
    } else {
      console.log('[HardwareProfile] User not logged in, skipping backend sync');
    }
  };

  const clearProfile = () => {
    console.log('[HardwareProfile] Clearing profile');
    setProfileState(null);
    localStorage.removeItem(STORAGE_KEY);
    setIsPersonalized(false);
  };

  const togglePersonalization = () => {
    setIsPersonalized(prev => {
      const newValue = !prev;
      console.log('[HardwareProfile] Toggling personalization:', newValue);
      return newValue;
    });
  };

  return (
    <HardwareProfileContext.Provider
      value={{
        profile,
        isPersonalized,
        setProfile,
        clearProfile,
        togglePersonalization
      }}
    >
      {children}
    </HardwareProfileContext.Provider>
  );
}

/**
 * Hook to access hardware profile context
 * @throws Error if used outside HardwareProfileProvider
 */
export function useHardwareProfile(): HardwareProfileContextValue {
  const context = useContext(HardwareProfileContext);
  if (!context) {
    throw new Error('useHardwareProfile must be used within HardwareProfileProvider');
  }
  return context;
}
