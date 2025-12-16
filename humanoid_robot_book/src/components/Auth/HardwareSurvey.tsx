import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './AuthForm.module.css';

interface HardwareSurveyData {
  hardwareType: 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null;
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  additionalNotes?: string;
}

const HardwareSurvey: React.FC = () => {
  const { user, isLoading: authLoading, updateHardwareProfile } = useAuth();
  const [hardwareSurvey, setHardwareSurvey] = useState<HardwareSurveyData>({
    hardwareType: null,
    gpuModel: '',
    cpuModel: '',
    ramSize: undefined,
    osType: '',
    additionalNotes: '',
  });
  const [error, setError] = useState<string | null>(null);
  const [isUpdating, setIsUpdating] = useState(false);
  const [isSuccess, setIsSuccess] = useState(false);

  // Load existing hardware profile if available
  useEffect(() => {
    if (user?.hardwareProfile) {
      setHardwareSurvey({
        hardwareType: user.hardwareProfile.hardwareType,
        gpuModel: user.hardwareProfile.gpuModel || '',
        cpuModel: user.hardwareProfile.cpuModel || '',
        ramSize: user.hardwareProfile.ramSize || undefined,
        osType: user.hardwareProfile.osType || '',
        additionalNotes: user.hardwareProfile.additionalNotes || '',
      });
    }
  }, [user]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsUpdating(true);
    setIsSuccess(false);

    try {
      await updateHardwareProfile({
        hardwareType: hardwareSurvey.hardwareType,
        gpuModel: hardwareSurvey.gpuModel || undefined,
        cpuModel: hardwareSurvey.cpuModel || undefined,
        ramSize: hardwareSurvey.ramSize || undefined,
        osType: hardwareSurvey.osType || undefined,
        additionalNotes: hardwareSurvey.additionalNotes || undefined,
      });
      setIsSuccess(true);
      setTimeout(() => setIsSuccess(false), 3000); // Hide success message after 3 seconds
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while updating your profile');
    } finally {
      setIsUpdating(false);
    }
  };

  const handleHardwareTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setHardwareSurvey({
      ...hardwareSurvey,
      hardwareType: e.target.value as 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null,
    });
  };

  if (authLoading) {
    return (
      <div className={styles.authContainer}>
        <p>Loading profile...</p>
      </div>
    );
  }

  if (!user) {
    return (
      <div className={styles.authContainer}>
        <p>Please sign in to update your hardware profile.</p>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <h2>Update Hardware Profile</h2>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.hardwareSurveySection}>
          <p className={styles.subtitle}>Update your hardware information to receive personalized content recommendations</p>

          <div className={styles.formGroup}>
            <label htmlFor="hardwareType">Primary Hardware Type</label>
            <select
              id="hardwareType"
              value={hardwareSurvey.hardwareType || ''}
              onChange={handleHardwareTypeChange}
              required
              className={styles.input}
            >
              <option value="">Select your hardware type</option>
              <option value="gpu_workstation">GPU Workstation (RTX 4090, Multiple GPUs, etc.)</option>
              <option value="edge_device">Edge Device (NVIDIA Jetson, etc.)</option>
              <option value="cloud_mac">Cloud/Mac (No dedicated GPU)</option>
            </select>
          </div>

          {hardwareSurvey.hardwareType === 'gpu_workstation' && (
            <>
              <div className={styles.formGroup}>
                <label htmlFor="gpuModel">GPU Model</label>
                <input
                  type="text"
                  id="gpuModel"
                  value={hardwareSurvey.gpuModel || ''}
                  onChange={(e) => setHardwareSurvey({...hardwareSurvey, gpuModel: e.target.value})}
                  placeholder="e.g., RTX 4090, RTX 6000 Ada, etc."
                  className={styles.input}
                />
              </div>
              <div className={styles.formGroup}>
                <label htmlFor="ramSize">RAM Size (GB)</label>
                <input
                  type="number"
                  id="ramSize"
                  value={hardwareSurvey.ramSize || ''}
                  onChange={(e) => setHardwareSurvey({...hardwareSurvey, ramSize: Number(e.target.value)})}
                  placeholder="e.g., 64, 128"
                  className={styles.input}
                />
              </div>
            </>
          )}

          {hardwareSurvey.hardwareType === 'edge_device' && (
            <div className={styles.formGroup}>
              <label htmlFor="gpuModel">Edge Device Model</label>
              <input
                type="text"
                id="gpuModel"
                value={hardwareSurvey.gpuModel || ''}
                onChange={(e) => setHardwareSurvey({...hardwareSurvey, gpuModel: e.target.value})}
                placeholder="e.g., Jetson Orin, Jetson Nano, etc."
                className={styles.input}
              />
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="cpuModel">CPU Model</label>
            <input
              type="text"
              id="cpuModel"
              value={hardwareSurvey.cpuModel || ''}
              onChange={(e) => setHardwareSurvey({...hardwareSurvey, cpuModel: e.target.value})}
              placeholder="e.g., Intel i9-13900K, AMD Ryzen 7950X, Apple M2, etc."
              className={styles.input}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="osType">Operating System</label>
            <select
              id="osType"
              value={hardwareSurvey.osType || ''}
              onChange={(e) => setHardwareSurvey({...hardwareSurvey, osType: e.target.value})}
              className={styles.input}
            >
              <option value="">Select OS</option>
              <option value="windows">Windows</option>
              <option value="linux">Linux</option>
              <option value="macos">macOS</option>
              <option value="other">Other</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="additionalNotes">Additional Notes</label>
            <textarea
              id="additionalNotes"
              value={hardwareSurvey.additionalNotes || ''}
              onChange={(e) => setHardwareSurvey({...hardwareSurvey, additionalNotes: e.target.value})}
              placeholder="Any other relevant hardware information..."
              rows={3}
              className={styles.textarea}
            />
          </div>
        </div>

        {error && <div className={styles.error}>{error}</div>}
        {isSuccess && (
          <div className={styles.success}>
            Hardware profile updated successfully!
          </div>
        )}

        <button
          type="submit"
          disabled={isUpdating}
          className={`${styles.button} ${styles.primaryButton}`}
        >
          {isUpdating ? 'Updating...' : 'Update Profile'}
        </button>
      </form>
    </div>
  );
};

export default HardwareSurvey;