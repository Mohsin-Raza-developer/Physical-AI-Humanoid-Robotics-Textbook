// Utility functions for password validation based on requirements

export interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

/**
 * Validates password based on specified requirements:
 * - 8+ characters
 * - Mixed case (both uppercase and lowercase)
 * - Contains at least one number
 * - Contains at least one special character
 */
export function validatePassword(password: string): ValidationResult {
  const errors: string[] = [];

  // Check minimum length
  if (password.length < 8) {
    errors.push('Password must be at least 8 characters long');
  }

  // Check for uppercase letter
  if (!/[A-Z]/.test(password)) {
    errors.push('Password must contain at least one uppercase letter');
  }

  // Check for lowercase letter
  if (!/[a-z]/.test(password)) {
    errors.push('Password must contain at least one lowercase letter');
  }

  // Check for number
  if (!/\d/.test(password)) {
    errors.push('Password must contain at least one number');
  }

  // Check for special character
  if (!/[!@#$%^&*(),.?":{}|<>[\]\\;'`~\-=_+]/.test(password)) {
    errors.push('Password must contain at least one special character');
  }

  // Additional security checks to prevent common passwords
  const commonPasswords = [
    'password', '12345678', 'qwerty123', 'letmein123',
    'welcome123', 'admin123', 'user123', 'test123'
  ];

  if (commonPasswords.some(common => password.toLowerCase().includes(common))) {
    errors.push('Password contains common patterns that are easy to guess');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validates email format
 */
export function validateEmail(email: string): ValidationResult {
  const errors: string[] = [];

  if (!email) {
    errors.push('Email is required');
  } else {
    // Basic email regex pattern
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      errors.push('Email format is invalid');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validates onboarding question inputs
 */
export function validateOnboardingQuestions(
  softwareLevel: string | undefined,
  hardwareAccess: string | undefined
): ValidationResult {
  const errors: string[] = [];

  if (!softwareLevel) {
    errors.push('Software background level is required');
  } else if (!['Beginner', 'Intermediate', 'Advanced'].includes(softwareLevel)) {
    errors.push('Software background level must be one of: Beginner, Intermediate, Advanced');
  }

  if (!hardwareAccess) {
    errors.push('Hardware access is required');
  } else if (!['Laptop/Cloud', 'RTX GPU', 'Jetson Edge', 'Physical Robot'].includes(hardwareAccess)) {
    errors.push('Hardware access must be one of: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Generic validation function that combines multiple validators
 */
export function validateRegistration(
  email: string,
  password: string,
  firstName: string,
  lastName: string,
  softwareLevel: string,
  hardwareAccess: string
): ValidationResult {
  const emailValidation = validateEmail(email);
  const passwordValidation = validatePassword(password);
  const firstNameValidation = validateRequired(firstName, 'First Name');
  const lastNameValidation = validateRequired(lastName, 'Last Name');
  const onboardingValidation = validateOnboardingQuestions(softwareLevel, hardwareAccess);

  const allErrors = [
    ...emailValidation.errors,
    ...passwordValidation.errors,
    ...firstNameValidation.errors,
    ...lastNameValidation.errors,
    ...onboardingValidation.errors
  ];

  return {
    isValid: allErrors.length === 0,
    errors: allErrors
  };
}

/**
 * Validates a new user object against all requirements
 */
export function validateNewUser(userData: {
  email: string;
  password: string;
  firstName: string;
  lastName: string;
  softwareLevel: string;
  hardwareAccess: string;
}): ValidationResult {
  return validateRegistration(
    userData.email,
    userData.password,
    userData.firstName,
    userData.lastName,
    userData.softwareLevel,
    userData.hardwareAccess
  );
}

/**
 * Validates a new password for a password reset operation
 */
export function validateNewPassword(newPassword: string, confirmPassword?: string): ValidationResult {
  const passwordValidation = validatePassword(newPassword);
  const errors = [...passwordValidation.errors];

  // If confirm password is provided, check if it matches
  if (confirmPassword !== undefined && newPassword !== confirmPassword) {
    errors.push('Passwords do not match');
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

// Additional validation helpers for various input types

/**
 * Validates that an input is not empty/null/undefined
 */
export function validateRequired(value: any, fieldName: string): ValidationResult {
  if (value === null || value === undefined || value === '') {
    return {
      isValid: false,
      errors: [`${fieldName} is required`]
    };
  }

  return {
    isValid: true,
    errors: []
  };
}

/**
 * Sanitizes user inputs to prevent injection attacks
 */
export function sanitizeInput(input: any): any {
  if (typeof input === 'string') {
    // Import the sanitizeString function from the sanitize utility
    // In a real implementation, we'd import from the sanitize module
    return input
      .replace(/\0/g, '') // Remove null bytes
      .replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, '') // Remove control characters
      .replace(/&/g, '&amp;') // Escape HTML
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#x27;');
  } else if (typeof input === 'object' && input !== null) {
    // Recursively sanitize object properties
    const sanitized: any = {};
    for (const [key, value] of Object.entries(input)) {
      sanitized[key] = sanitizeInput(value);
    }
    return sanitized;
  }
  return input;
}

/**
 * Validates that a string does not exceed a certain length
 */
export function validateMaxLength(value: string, maxLength: number, fieldName: string): ValidationResult {
  if (value.length > maxLength) {
    return {
      isValid: false,
      errors: [`${fieldName} must not exceed ${maxLength} characters`]
    };
  }

  return {
    isValid: true,
    errors: []
  };
}

/**
 * Validates that a value is one of the allowed options
 */
export function validateInclusion(value: string, allowedValues: string[], fieldName: string): ValidationResult {
  if (!allowedValues.includes(value)) {
    return {
      isValid: false,
      errors: [`${fieldName} must be one of: ${allowedValues.join(', ')}`]
    };
  }

  return {
    isValid: true,
    errors: []
  };
}