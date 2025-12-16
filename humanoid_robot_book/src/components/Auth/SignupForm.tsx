import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './AuthForm.module.css'; // We'll create this CSS file

interface HardwareSurveyData {
  hardwareType: 'gpu_workstation' | 'edge_device' | 'cloud_mac' | '';
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  additionalNotes?: string;
}

const SignupForm: React.FC = () => {
  const { signUp } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [hardwareSurvey, setHardwareSurvey] = useState<HardwareSurveyData>({
    hardwareType: '',
    gpuModel: '',
    cpuModel: '',
    ramSize: undefined,
    osType: '',
    additionalNotes: '',
  });
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsLoading(true);

    try {
      await signUp(
        email,
        password,
        name,
        {
          hardwareType: hardwareSurvey.hardwareType as 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null,
          gpuModel: hardwareSurvey.gpuModel || undefined,
          cpuModel: hardwareSurvey.cpuModel || undefined,
          ramSize: hardwareSurvey.ramSize || undefined,
          osType: hardwareSurvey.osType || undefined,
          additionalNotes: hardwareSurvey.additionalNotes || undefined,
        }
      );
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during sign up');
    } finally {
      setIsLoading(false);
    }
  };

  const handleHardwareTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setHardwareSurvey({
      ...hardwareSurvey,
      hardwareType: e.target.value as 'gpu_workstation' | 'edge_device' | 'cloud_mac' | '',
    });
  };

  return (
    <div className={styles.authContainer}>
      <h2>Create Account</h2>
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.formGroup}>
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            className={styles.input}
          />
        </div>

        <div className={styles.hardwareSurveySection}>
          <h3>Tell us about your hardware setup</h3>
          <p className={styles.subtitle}>This helps us personalize content for your specific setup</p>

          <div className={styles.formGroup}>
            <label htmlFor="hardwareType">Primary Hardware Type</label>
            <select
              id="hardwareType"
              value={hardwareSurvey.hardwareType}
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

        <button
          type="submit"
          disabled={isLoading}
          className={`${styles.button} ${styles.primaryButton}`}
        >
          {isLoading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;