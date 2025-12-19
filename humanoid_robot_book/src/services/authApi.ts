/**
 * Authentication API Service
 * Handles communication with the FastAPI backend for authentication
 */

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://humanoid-robotics-backend.vercel.app'  // Backend API URL
  : 'http://localhost:8000';  // Local development

export interface SignupRequest {
  email: string;
  password: string;
  name: string;
  // Software background (Feature 008)
  software_experience?: string | null;
  programming_languages?: string[] | null;
  // Hardware background
  hardware_type?: string | null;
  hardware_details?: any;
  hardware_experience?: boolean;
}

export interface SignupResponse {
  success: boolean;
  message: string;
  user: {
    id: string;
    email: string;
    name: string;
  };
}

export interface LoginRequest {
  email: string;
  password: string;
}

export interface LoginResponse {
  success: boolean;
  message: string;
  user: {
    id: string;
    email: string;
    name: string;
  };
}

export async function signupApi(request: SignupRequest): Promise<SignupResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: request.email,
        password: request.password,
        name: request.name,
        // Software background (Feature 008)
        software_experience: request.software_experience,
        programming_languages: request.programming_languages,
        // Hardware background
        hardware_type: request.hardware_type,
        hardware_details: request.hardware_details,
        hardware_experience: request.hardware_experience
      }),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP ${response.status}: ${response.statusText}`);
    }

    const data: SignupResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Signup API error:', error);
    throw error;
  }
}

export async function loginApi(request: LoginRequest): Promise<LoginResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: request.email,
        password: request.password
      }),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP ${response.status}: ${response.statusText}`);
    }

    const data: LoginResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Login API error:', error);
    throw error;
  }
}

export async function getUserProfile(userId: string) {
  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/user/${userId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP ${response.status}: ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Get user profile API error:', error);
    throw error;
  }
}

export async function updateUserProfile(userId: string, updateData: any) {
  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/user/${userId}`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(updateData),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP ${response.status}: ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Update user profile API error:', error);
    throw error;
  }
}

export async function logoutApi() {
  try {
    const response = await fetch(`${API_BASE_URL}/api/auth/logout`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP ${response.status}: ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Logout API error:', error);
    throw error;
  }
}