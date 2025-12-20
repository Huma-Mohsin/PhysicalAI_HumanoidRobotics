/**
 * Hardware profile types for personalization system
 * Feature 005: Interactive Personalization System
 */

export type HardwareType = 'gpu_workstation' | 'edge_device' | 'cloud_mac';

export interface HardwareProfile {
  type: HardwareType;
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  selectedAt: Date;
}

export interface HardwareProfileContextValue {
  profile: HardwareProfile | null;
  isPersonalized: boolean;
  setProfile: (profile: HardwareProfile) => Promise<void>;
  clearProfile: () => void;
  togglePersonalization: () => void;
}

/**
 * Hardware type display labels
 */
export const HARDWARE_LABELS: Record<HardwareType, string> = {
  gpu_workstation: 'GPU Workstation (RTX 4090+)',
  edge_device: 'Jetson Orin Nano',
  cloud_mac: 'Cloud / Mac',
};

/**
 * Hardware type descriptions for survey modal
 */
export const HARDWARE_DESCRIPTIONS: Record<HardwareType, string> = {
  gpu_workstation: 'RTX 4070 Ti or higher, local Isaac Sim',
  edge_device: 'Jetson Orin Nano, Gazebo simulation',
  cloud_mac: 'No local GPU, cloud-based or Docker setup',
};
