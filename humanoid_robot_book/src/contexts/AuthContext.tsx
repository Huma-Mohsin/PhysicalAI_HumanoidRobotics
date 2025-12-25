import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { createAuthClient } from 'better-auth/react';
import { signupApi, loginApi, getUserProfile, updateUserProfile, logoutApi } from '../services/authApi';

// Define types for user and hardware profile
interface HardwareProfile {
  id: string;
  userId: string;
  hardwareType: 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null;
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  additionalNotes?: string;
  createdAt: string;
  updatedAt: string;
}

interface UserProfile {
  id: string;
  email: string;
  name: string;
  // Software background (Feature 008)
  softwareExperience?: string;
  programmingLanguages?: string[];
  // Hardware profile
  hardwareProfile: HardwareProfile | null;
  hardwareType?: string;
  hardwareExperience?: boolean;
  createdAt: string;
  updatedAt: string;
}

interface SignUpData {
  // Software background
  softwareExperience?: string;
  programmingLanguages?: string[];
  // Hardware background
  hardwareType?: 'gpu_workstation' | 'edge_device' | 'cloud_mac' | null;
  hardwareExperience?: boolean;
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  additionalNotes?: string;
}

interface AuthContextType {
  user: UserProfile | null;
  isLoading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, name: string, signUpData: SignUpData) => Promise<void>;
  signOut: () => Promise<void>;
  updateHardwareProfile: (hardwareData: Omit<HardwareProfile, 'id' | 'userId' | 'createdAt' | 'updatedAt'>) => Promise<void>;
}

// Initialize Better-Auth client
const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === 'production'
    ? 'https://humanoid-robotics-backend.vercel.app'  // Backend API URL
    : 'http://localhost:8000',
  fetch: globalThis.fetch,
});

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<UserProfile | null>(() => {
    // Initialize from localStorage on mount
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem('userProfile');
      return stored ? JSON.parse(stored) : null;
    }
    return null;
  });
  const [isLoading, setIsLoading] = useState(true);

  // Save user to localStorage whenever it changes
  useEffect(() => {
    if (typeof window !== 'undefined') {
      if (user) {
        localStorage.setItem('userProfile', JSON.stringify(user));
      } else {
        localStorage.removeItem('userProfile');
      }
    }
  }, [user]);

  // Check session on component mount
  useEffect(() => {
    const checkSession = async () => {
      // If user already loaded from localStorage, skip backend check initially
      if (user) {
        setIsLoading(false);
        return;
      }

      try {
        // Call backend's get-session endpoint which checks the session cookie
        const API_BASE_URL = process.env.NODE_ENV === 'production'
          ? 'https://humanoid-robotics-backend.vercel.app'
          : 'http://localhost:8000';

        const response = await fetch(`${API_BASE_URL}/api/auth/get-session`, {
          method: 'GET',
          credentials: 'include', // Important: Send cookies
        });

        if (response.ok) {
          const data = await response.json();
          if (data.success && data.user) {
            // Set user from session data
            const userProfile: UserProfile = {
              id: data.user.id,
              email: data.user.email,
              name: data.user.name,
              softwareExperience: data.user.softwareExperience,
              programmingLanguages: data.user.programmingLanguages || [],
              hardwareType: data.user.hardwareType,
              hardwareExperience: data.user.hardwareExperience,
              hardwareProfile: data.hardware_profile ? {
                id: 'temp-id',
                userId: data.user.id,
                hardwareType: data.hardware_profile.type,
                gpuModel: data.hardware_profile.details?.gpu_model,
                cpuModel: data.hardware_profile.details?.cpu_model,
                ramSize: data.hardware_profile.details?.ram_size,
                osType: data.hardware_profile.details?.os_type,
                additionalNotes: data.hardware_profile.details?.additional_notes,
                createdAt: new Date().toISOString(),
                updatedAt: new Date().toISOString(),
              } : null,
              createdAt: new Date().toISOString(),
              updatedAt: new Date().toISOString(),
            };
            setUser(userProfile);
          }
        } else if (response.status === 401) {
          // 401 is expected for users who aren't logged in - silently continue
          // No error logging needed
        }
      } catch (error) {
        // Only log errors that aren't network/connection issues
        if (error instanceof TypeError && error.message.includes('fetch')) {
          // Network error - silently fail (backend might be down)
        } else {
          console.error('Error checking session:', error);
        }
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();
  }, [user]);

  const fetchUserProfile = async (userId: string): Promise<UserProfile> => {
    try {
      const response = await getUserProfile(userId);
      // Format the response to match UserProfile interface
      return {
        id: response.user.id,
        email: response.user.email,
        name: response.user.name,
        hardwareProfile: response.user.hardware_details ? {
          id: 'temp-id',
          userId: response.user.id,
          hardwareType: response.user.hardware_type,
          gpuModel: response.user.hardware_details.gpu_model,
          cpuModel: response.user.hardware_details.cpu_model,
          ramSize: response.user.hardware_details.ram_size,
          osType: response.user.hardware_details.os_type,
          additionalNotes: response.user.hardware_details.additional_notes,
          createdAt: response.user.created_at,
          updatedAt: response.user.updated_at,
        } : null,
        createdAt: response.user.created_at,
        updatedAt: response.user.updated_at,
      };
    } catch (error) {
      console.error('Error fetching user profile:', error);
      // Return a basic profile if API call fails
      return {
        id: userId,
        email: 'user@example.com',
        name: 'User',
        hardwareProfile: null,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      };
    }
  };

  const signIn = async (email: string, password: string) => {
    setIsLoading(true);
    try {
      // Use direct API call instead of Better-Auth client method
      const response = await loginApi({
        email,
        password
      });

      if (response.success && response.user) {
        // Create user profile from response - NOW WITH BACKGROUND DATA
        const userProfile = {
          id: response.user.id,
          email: response.user.email,
          name: response.user.name,
          // Software background (Feature 008)
          softwareExperience: response.user.softwareExperience,
          programmingLanguages: response.user.programmingLanguages || [],
          // Hardware background
          hardwareType: response.user.hardwareType,
          hardwareExperience: response.user.hardwareExperience,
          hardwareProfile: null, // This will be constructed from hardwareType
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
        setUser(userProfile);
      }
    } catch (error) {
      console.error('Sign in error:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const signUp = async (
    email: string,
    password: string,
    name: string,
    signUpData: SignUpData
  ) => {
    setIsLoading(true);
    try {
      // Use direct API call instead of Better-Auth client method
      const response = await signupApi({
        email,
        password,
        name,
        // Software background (Feature 008)
        software_experience: signUpData.softwareExperience || null,
        programming_languages: signUpData.programmingLanguages || null,
        // Hardware background
        hardware_type: signUpData.hardwareType || null,
        hardware_details: {
          gpu_model: signUpData.gpuModel,
          cpu_model: signUpData.cpuModel,
          ram_size: signUpData.ramSize,
          os_type: signUpData.osType,
          additional_notes: signUpData.additionalNotes
        },
        hardware_experience: signUpData.hardwareExperience || false
      });

      if (response.success && response.user) {
        // Create user profile from response - NOW WITH BACKGROUND DATA
        const userProfile = {
          id: response.user.id,
          email: response.user.email,
          name: response.user.name,
          // Software background (Feature 008)
          softwareExperience: response.user.softwareExperience || signUpData.softwareExperience,
          programmingLanguages: response.user.programmingLanguages || signUpData.programmingLanguages || [],
          // Hardware background
          hardwareType: response.user.hardwareType || signUpData.hardwareType,
          hardwareExperience: response.user.hardwareExperience ?? signUpData.hardwareExperience ?? false,
          hardwareProfile: signUpData.hardwareType ? {
            id: 'temp-id', // This would be generated by your backend
            userId: response.user.id,
            hardwareType: signUpData.hardwareType,
            gpuModel: signUpData.gpuModel,
            cpuModel: signUpData.cpuModel,
            ramSize: signUpData.ramSize,
            osType: signUpData.osType,
            additionalNotes: signUpData.additionalNotes,
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString(),
          } : null,
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        };
        setUser(userProfile);
      }
    } catch (error) {
      console.error('Sign up error:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const createUserProfile = async (
    userId: string,
    name: string,
    email: string,
    hardwareData: Omit<HardwareProfile, 'id' | 'userId' | 'createdAt' | 'updatedAt'>
  ): Promise<UserProfile> => {
    // This would be an API call to your backend to create user profile
    // For now, returning a mock profile
    return {
      id: userId,
      email,
      name,
      hardwareProfile: {
        ...hardwareData,
        id: 'temp-id', // This would be generated by your backend
        userId,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
    };
  };

  const signOut = async () => {
    setIsLoading(true);
    try {
      await logoutApi();
      setUser(null);
    } catch (error) {
      console.error('Sign out error:', error);
      // Even if the API call fails, clear the local user state
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  };

  const updateHardwareProfile = async (
    hardwareData: Omit<HardwareProfile, 'id' | 'userId' | 'createdAt' | 'updatedAt'>
  ) => {
    if (!user) {
      throw new Error('User not authenticated');
    }

    setIsLoading(true);
    try {
      // Update hardware profile via API call
      const response = await updateUserProfile(user.id, {
        hardware_type: hardwareData.hardwareType,
        hardware_details: {
          gpu_model: hardwareData.gpuModel,
          cpu_model: hardwareData.cpuModel,
          ram_size: hardwareData.ramSize,
          os_type: hardwareData.osType,
          additional_notes: hardwareData.additionalNotes
        }
      });

      // Update the user state with the new hardware profile
      setUser({
        ...user,
        hardwareProfile: {
          ...hardwareData,
          id: 'temp-id',
          userId: user.id,
          createdAt: user.createdAt,
          updatedAt: new Date().toISOString(),
        }
      });
    } catch (error) {
      console.error('Update hardware profile error:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isLoading,
        signIn,
        signUp,
        signOut,
        updateHardwareProfile,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};