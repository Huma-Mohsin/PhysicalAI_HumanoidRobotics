/**
 * Authentication Hook
 *
 * Manages user authentication state, signup, signin, and hardware profile updates.
 */

import { useState, useEffect } from 'react';

export interface HardwareProfile {
  id: string;
  user_id: string;
  os_type?: string;
  gpu_model?: string;
  environment_preference: 'workstation' | 'cloud' | 'mac';
  language_preference: 'en' | 'ur';
  is_workstation_capable: boolean;
  created_at: string;
  updated_at?: string;
}

export interface User {
  id: string;
  email: string;
  created_at: string;
  hardware_profile?: HardwareProfile;
}

export interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  token: string | null;
}

export interface SignUpData {
  email: string;
  password: string;
  os_type?: string;
  gpu_model?: string;
  environment_preference?: 'workstation' | 'cloud' | 'mac';
  language_preference?: 'en' | 'ur';
}

export interface SignInData {
  email: string;
  password: string;
}

export interface UpdateHardwareProfileData {
  os_type?: string;
  gpu_model?: string;
  environment_preference?: 'workstation' | 'cloud' | 'mac';
  language_preference?: 'en' | 'ur';
}

const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const useAuth = () => {
  const [authState, setAuthState] = useState<AuthState>({
    user: null,
    isAuthenticated: false,
    isLoading: true,
    token: null,
  });

  // Load auth state from localStorage on mount
  useEffect(() => {
    const token = localStorage.getItem('token');
    const userStr = localStorage.getItem('user');

    if (token && userStr) {
      try {
        const user = JSON.parse(userStr);
        setAuthState({
          user,
          isAuthenticated: true,
          isLoading: false,
          token,
        });
      } catch (error) {
        console.error('Failed to parse stored user data:', error);
        localStorage.removeItem('token');
        localStorage.removeItem('user');
        setAuthState({ user: null, isAuthenticated: false, isLoading: false, token: null });
      }
    } else {
      setAuthState({ user: null, isAuthenticated: false, isLoading: false, token: null });
    }
  }, []);

  const signUp = async (data: SignUpData): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await fetch(`${apiUrl}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        return { success: false, error: error.detail || 'Signup failed' };
      }

      const result = await response.json();

      // Store token and user data
      localStorage.setItem('token', result.access_token);
      localStorage.setItem('user', JSON.stringify(result.user));

      setAuthState({
        user: result.user,
        isAuthenticated: true,
        isLoading: false,
        token: result.access_token,
      });

      return { success: true };
    } catch (error) {
      console.error('Signup error:', error);
      return { success: false, error: 'Network error. Please try again.' };
    }
  };

  const signIn = async (data: SignInData): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await fetch(`${apiUrl}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        return { success: false, error: error.detail || 'Sign in failed' };
      }

      const result = await response.json();

      // Store token and user data
      localStorage.setItem('token', result.access_token);
      localStorage.setItem('user', JSON.stringify(result.user));

      setAuthState({
        user: result.user,
        isAuthenticated: true,
        isLoading: false,
        token: result.access_token,
      });

      return { success: true };
    } catch (error) {
      console.error('Sign in error:', error);
      return { success: false, error: 'Network error. Please try again.' };
    }
  };

  const signOut = () => {
    localStorage.removeItem('token');
    localStorage.removeItem('user');

    setAuthState({
      user: null,
      isAuthenticated: false,
      isLoading: false,
      token: null,
    });
  };

  const updateHardwareProfile = async (
    data: UpdateHardwareProfileData
  ): Promise<{ success: boolean; error?: string }> => {
    if (!authState.token) {
      return { success: false, error: 'Not authenticated' };
    }

    try {
      const response = await fetch(`${apiUrl}/auth/hardware-profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${authState.token}`,
        },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        return { success: false, error: error.detail || 'Update failed' };
      }

      const updatedProfile = await response.json();

      // Update local user state
      const updatedUser = { ...authState.user!, hardware_profile: updatedProfile };
      localStorage.setItem('user', JSON.stringify(updatedUser));

      setAuthState({
        ...authState,
        user: updatedUser,
      });

      return { success: true };
    } catch (error) {
      console.error('Profile update error:', error);
      return { success: false, error: 'Network error. Please try again.' };
    }
  };

  const refreshUser = async (): Promise<{ success: boolean; error?: string }> => {
    if (!authState.token) {
      return { success: false, error: 'Not authenticated' };
    }

    try {
      const response = await fetch(`${apiUrl}/auth/me`, {
        headers: {
          'Authorization': `Bearer ${authState.token}`,
        },
      });

      if (!response.ok) {
        // Token might be expired
        signOut();
        return { success: false, error: 'Session expired' };
      }

      const user = await response.json();
      localStorage.setItem('user', JSON.stringify(user));

      setAuthState({
        ...authState,
        user,
      });

      return { success: true };
    } catch (error) {
      console.error('Refresh user error:', error);
      return { success: false, error: 'Network error' };
    }
  };

  return {
    ...authState,
    signUp,
    signIn,
    signOut,
    updateHardwareProfile,
    refreshUser,
  };
};
