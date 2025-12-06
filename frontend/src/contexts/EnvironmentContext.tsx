/**
 * Environment Context
 *
 * Manages environment mode (workstation/cloud/mac) based on user's hardware profile
 */

import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth, HardwareProfile } from '../hooks/useAuth';

export type EnvironmentMode = 'workstation' | 'cloud' | 'mac';

interface EnvironmentContextType {
  mode: EnvironmentMode;
  setMode: (mode: EnvironmentMode) => void;
  isWorkstationCapable: boolean;
  autoDetectedMode: EnvironmentMode;
}

const EnvironmentContext = createContext<EnvironmentContextType | undefined>(undefined);

export const EnvironmentProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { user } = useAuth();

  // Determine auto-detected mode from user's hardware profile
  const getAutoDetectedMode = (profile?: HardwareProfile): EnvironmentMode => {
    if (!profile) return 'cloud'; // Default for non-authenticated users
    return profile.environment_preference || 'cloud';
  };

  const isWorkstationCapable = user?.hardware_profile?.is_workstation_capable || false;
  const autoDetectedMode = getAutoDetectedMode(user?.hardware_profile);

  const [mode, setModeState] = useState<EnvironmentMode>(autoDetectedMode);

  // Update mode when user profile changes
  useEffect(() => {
    setModeState(autoDetectedMode);
  }, [autoDetectedMode]);

  // Persist mode preference to localStorage (for manual overrides)
  const setMode = (newMode: EnvironmentMode) => {
    setModeState(newMode);
    localStorage.setItem('environment_mode_override', newMode);
  };

  // Load override from localStorage on mount
  useEffect(() => {
    const override = localStorage.getItem('environment_mode_override') as EnvironmentMode;
    if (override && ['workstation', 'cloud', 'mac'].includes(override)) {
      setModeState(override);
    }
  }, []);

  return (
    <EnvironmentContext.Provider value={{ mode, setMode, isWorkstationCapable, autoDetectedMode }}>
      {children}
    </EnvironmentContext.Provider>
  );
};

export const useEnvironment = (): EnvironmentContextType => {
  const context = useContext(EnvironmentContext);
  if (!context) {
    throw new Error('useEnvironment must be used within EnvironmentProvider');
  }
  return context;
};
