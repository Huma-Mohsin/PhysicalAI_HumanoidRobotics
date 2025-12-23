import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './AuthForm.module.css'; // We'll create this CSS file

interface SoftwareBackgroundData {
  softwareExperience: 'beginner' | 'intermediate' | 'expert' | '';
  programmingLanguages: string[];
}

interface HardwareSurveyData {
  hardwareType: 'gpu_workstation' | 'edge_device' | 'cloud_mac' | '';
  hardwareExperience: boolean;
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
  const [softwareBackground, setSoftwareBackground] = useState<SoftwareBackgroundData>({
    softwareExperience: '',
    programmingLanguages: [],
  });
  const [hardwareSurvey, setHardwareSurvey] = useState<HardwareSurveyData>({
    hardwareType: '',
    hardwareExperience: false,
    gpuModel: '',
    cpuModel: '',
    ramSize: undefined,
    osType: '',
    additionalNotes: '',
  });
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [showDuplicateModal, setShowDuplicateModal] = useState(false);

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
          // Software background (Feature 008)
          softwareExperience: softwareBackground.softwareExperience || undefined,
          programmingLanguages: softwareBackground.programmingLanguages.length > 0
            ? softwareBackground.programmingLanguages
            : undefined,
          // Hardware background
          hardwareType: hardwareSurvey.hardwareType as 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null,
          hardwareExperience: hardwareSurvey.hardwareExperience,
          gpuModel: hardwareSurvey.gpuModel || undefined,
          cpuModel: hardwareSurvey.cpuModel || undefined,
          ramSize: hardwareSurvey.ramSize || undefined,
          osType: hardwareSurvey.osType || undefined,
          additionalNotes: hardwareSurvey.additionalNotes || undefined,
        }
      );
      // Navigate to home page after successful signup
      window.location.href = '/';
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An error occurred during sign up';

      // Check if it's a duplicate email error
      if (errorMessage.toLowerCase().includes('already exists') ||
          errorMessage.toLowerCase().includes('duplicate')) {
        setShowDuplicateModal(true);
      } else {
        setError(errorMessage);
      }
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

  const handleProgrammingLanguageToggle = (language: string) => {
    setSoftwareBackground(prev => ({
      ...prev,
      programmingLanguages: prev.programmingLanguages.includes(language)
        ? prev.programmingLanguages.filter(l => l !== language)
        : [...prev.programmingLanguages, language]
    }));
  };

  return (
    <>
      {/* Duplicate Email Modal */}
      {showDuplicateModal && (
        <div className={styles.modalOverlay} onClick={() => setShowDuplicateModal(false)}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalIcon}>
              <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <circle cx="12" cy="12" r="10" stroke="#f59e0b" strokeWidth="2"/>
                <path d="M12 8v4M12 16h.01" stroke="#f59e0b" strokeWidth="2" strokeLinecap="round"/>
              </svg>
            </div>
            <h3 className={styles.modalTitle}>User Already Exists</h3>
            <p className={styles.modalMessage}>
              An account with email <strong>{email}</strong> is already registered.
            </p>
            <p className={styles.modalMessage}>
              Please try logging in or use a different email address.
            </p>
            <div className={styles.modalActions}>
              <button
                onClick={() => window.location.href = '/login'}
                className={`${styles.button} ${styles.primaryButton}`}
              >
                Go to Login
              </button>
              <button
                onClick={() => setShowDuplicateModal(false)}
                className={`${styles.button} ${styles.secondaryButton}`}
              >
                Use Different Email
              </button>
            </div>
          </div>
        </div>
      )}

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

        <div className={styles.softwareBackgroundSection}>
          <h3>Software Background (Optional)</h3>
          <p className={styles.subtitle}>Help us personalize learning content for your experience level</p>

          <div className={styles.formGroup}>
            <label htmlFor="softwareExperience">Experience Level</label>
            <select
              id="softwareExperience"
              value={softwareBackground.softwareExperience}
              onChange={(e) => setSoftwareBackground({...softwareBackground, softwareExperience: e.target.value as 'beginner' | 'intermediate' | 'expert' | ''})}
              className={styles.input}
            >
              <option value="">Select your experience level</option>
              <option value="beginner">Beginner (New to robotics/programming)</option>
              <option value="intermediate">Intermediate (Some experience with code)</option>
              <option value="expert">Expert (Experienced developer)</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label>Programming Languages (check all that apply)</label>
            <div className={styles.checkboxGroup}>
              {['Python', 'JavaScript', 'C++', 'ROS 2', 'None'].map(language => (
                <label key={language} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={softwareBackground.programmingLanguages.includes(language)}
                    onChange={() => handleProgrammingLanguageToggle(language)}
                    className={styles.checkbox}
                  />
                  {language}
                </label>
              ))}
            </div>
          </div>
        </div>

        <div className={styles.hardwareSurveySection}>
          <h3>Hardware Background (Optional)</h3>
          <p className={styles.subtitle}>This helps us personalize content for your specific setup</p>

          <div className={styles.formGroup}>
            <label htmlFor="hardwareType">Primary Hardware Type</label>
            <select
              id="hardwareType"
              value={hardwareSurvey.hardwareType}
              onChange={handleHardwareTypeChange}
              className={styles.input}
            >
              <option value="">Select your hardware type</option>
              <option value="gpu_workstation">GPU Workstation (RTX 4090, Multiple GPUs, etc.)</option>
              <option value="edge_device">Edge Device (NVIDIA Jetson, etc.)</option>
              <option value="cloud_mac">Cloud/Mac (No dedicated GPU)</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={hardwareSurvey.hardwareExperience}
                onChange={(e) => setHardwareSurvey({...hardwareSurvey, hardwareExperience: e.target.checked})}
                className={styles.checkbox}
              />
              I have experience with robotics hardware
            </label>
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
    </>
  );
};

export default SignupForm;