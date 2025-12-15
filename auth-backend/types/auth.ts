// Type definitions for User entity based on data-model.md

export interface User {
  id: string;
  email: string;
  first_name: string;
  last_name: string;
  password_hash: string;
  software_level: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
  created_at: Date;
  updated_at: Date;
  last_login_at?: Date | null;
  email_verified: boolean;
  verification_token?: string | null;
  is_active: boolean;
}

export interface CreateUserInput {
  email: string;
  first_name: string;
  last_name: string;
  password_hash: string;
  software_level: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
  email_verified?: boolean;
  verification_token?: string | null;
}

export interface UpdateUserProfileInput {
  software_level?: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access?: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
}

// Type definitions for Password Reset Token entity based on data-model.md

export interface PasswordResetToken {
  id: string;
  user_id: string;
  token: string;
  expires_at: Date;
  used: boolean;
  created_at: Date;
}

// Additional general types for authentication

export interface AuthResponse {
  success: boolean;
  message: string;
  data?: any;
  errors?: Array<{
    field: string;
    message: string;
  }>;
}

export interface LoginRequest {
  email: string;
  password: string;
}

export interface LoginResponse extends AuthResponse {
  data?: {
    userId: string;
    email: string;
    firstName?: string;
    lastName?: string;
    sessionTimeout: number; // Session timeout in milliseconds
  };
}

export interface RegistrationRequest {
  firstName: string;
  lastName: string;
  email: string;
  password: string;
  softwareLevel: 'Beginner' | 'Intermediate' | 'Advanced';
  hardwareAccess: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
}

export interface RegistrationResponse extends AuthResponse {
  data?: {
    userId: string;
    email: string;
    createdAt: string;
  };
}

export interface PasswordResetRequest {
  email: string;
}

export interface PasswordResetCompleteRequest {
  token: string;
  newPassword: string;
}

export interface PasswordResetValidateResponse {
  success: boolean;
  valid: boolean;
  expiresAt?: string;
  message?: string;
}

export interface ProfileResponse extends AuthResponse {
  data?: User;
}