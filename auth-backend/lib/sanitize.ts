/**
 * Utility functions for input sanitization to prevent injection attacks
 */

// Sanitize a string to prevent XSS and injection attacks
export function sanitizeString(input: string): string {
  if (typeof input !== 'string') {
    return '';
  }
  
  // Remove null bytes and control characters
  let sanitized = input.replace(/\0/g, '');
  
  // Remove control characters (except tab, newline, carriage return)
  sanitized = sanitized.replace(/[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]/g, '');
  
  // Escape special HTML characters to prevent XSS
  sanitized = sanitized
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#x27;');
  
  return sanitized;
}

// Sanitize email input
export function sanitizeEmail(email: string): string {
  if (typeof email !== 'string') {
    return '';
  }
  
  // Basic email sanitization - trim whitespace and lowercase
  const sanitized = email.trim().toLowerCase();
  
  // Additional validation can be added here if needed
  return sanitized;
}

// Sanitize user input for SQL queries (basic implementation)
export function sanitizeForSQL(input: string): string {
  if (typeof input !== 'string') {
    return '';
  }
  
  // Remove or escape SQL metacharacters
  return input
    .replace(/'/g, "''")  // Escape single quotes
    .replace(/--/g, '')   // Remove SQL comments
    .replace(/;/g, '')    // Remove semicolons
    .replace(/\/\*/g, '') // Remove /* comments
    .replace(/\*\//g, ''); // Remove */ comments
}

// Sanitize input for shell commands (to prevent command injection)
export function sanitizeForShell(input: string): string {
  if (typeof input !== 'string') {
    return '';
  }
  
  // Remove shell metacharacters
  return input.replace(/[;&|`$(){}[\]\\<>~*?!]/g, '');
}

// Validate and sanitize object properties
export function sanitizeObject<T extends Record<string, any>>(obj: T, allowedFields?: string[]): T {
  const sanitizedObj = {} as T;
  
  for (const [key, value] of Object.entries(obj)) {
    // If allowed fields are specified, only sanitize those
    if (allowedFields && !allowedFields.includes(key)) {
      continue;
    }
    
    if (typeof value === 'string') {
      sanitizedObj[key as keyof T] = sanitizeString(value) as T[keyof T];
    } else if (typeof value === 'object' && value !== null) {
      // Recursively sanitize nested objects
      sanitizedObj[key as keyof T] = sanitizeObject(value, allowedFields) as T[keyof T];
    } else {
      // For non-string values, just pass them through
      sanitizedObj[key as keyof T] = value;
    }
  }
  
  return sanitizedObj;
}