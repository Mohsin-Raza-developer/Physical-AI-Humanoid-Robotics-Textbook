# API Documentation: Docusaurus Authentication System

This document provides detailed information about the API endpoints available in the Docusaurus Authentication System.

## Authentication Endpoints

### Register User
- **Endpoint**: `POST /api/auth/register`
- **Description**: Register a new user with email, password, and onboarding questions
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "SecurePass123!",
    "softwareLevel": "Intermediate",
    "hardwareAccess": "RTX GPU"
  }
  ```
- **Validation**:
  - Email: Required, valid email format
  - Password: Required, 8+ chars, mixed case, number, special char
  - SoftwareLevel: Required, one of ["Beginner", "Intermediate", "Advanced"]
  - HardwareAccess: Required, one of ["Laptop/Cloud", "RTX GPU", "Jetson Edge", "Physical Robot"]
- **Response (Success 201)**:
  ```json
  {
    "success": true,
    "message": "User registered successfully",
    "data": {
      "userId": "uuid-string",
      "email": "user@example.com",
      "createdAt": "2023-01-01T00:00:00.000Z"
    }
  }
  ```
- **Response (Validation Error 400)**:
  ```json
  {
    "success": false,
    "message": "Validation failed",
    "errors": [
      {
        "field": "email",
        "message": "Invalid email format"
      }
    ]
  }
  ```
- **Response (Conflict 409)**:
  ```json
  {
    "success": false,
    "message": "Email already registered"
  }
  ```

### Login User
- **Endpoint**: `POST /api/auth/login`
- **Description**: Authenticate user with email and password
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "SecurePass123!"
  }
  ```
- **Validation**:
  - Email: Required, valid email format
  - Password: Required
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "message": "Login successful",
    "data": {
      "userId": "uuid-string",
      "email": "user@example.com",
      "sessionTimeout": "7776000000"  // 30 days in milliseconds
    }
  }
  ```
- **Response (Unauthorized 401)**:
  ```json
  {
    "success": false,
    "message": "Invalid email or password"
  }
  ```

### Logout User
- **Endpoint**: `POST /api/auth/logout`
- **Description**: Terminate user session
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "message": "Logged out successfully"
  }
  ```

### Get User Profile
- **Endpoint**: `GET /api/users/profile?userId={userId}`
- **Description**: Retrieve current authenticated user's profile
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "data": {
      "id": "uuid-string",
      "email": "user@example.com",
      "softwareLevel": "Intermediate",
      "hardwareAccess": "RTX GPU",
      "createdAt": "2023-01-01T00:00:00.000Z",
      "lastLoginAt": "2023-01-01T12:00:00.000Z",
      "emailVerified": true,
      "isActive": true,
      "updatedAt": "2023-01-01T12:00:00.000Z"
    }
  }
  ```

## Password Management Endpoints

### Request Password Reset
- **Endpoint**: `POST /api/auth/reset-password/request`
- **Description**: Initiate password reset process by sending reset email
- **Request Body**:
  ```json
  {
    "email": "user@example.com"
  }
  ```
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "message": "Password reset email sent if email exists"
  }
  ```

### Validate Reset Token
- **Endpoint**: `GET /api/auth/reset-password/validate?token={token}`
- **Description**: Check if reset token is valid and not expired
- **Response (Valid 200)**:
  ```json
  {
    "success": true,
    "valid": true,
    "expiresAt": "2023-01-01T13:00:00.000Z"
  }
  ```
- **Response (Invalid 400)**:
  ```json
  {
    "success": false,
    "valid": false,
    "message": "Invalid or expired token"
  }
  ```

### Complete Password Reset
- **Endpoint**: `POST /api/auth/reset-password/complete`
- **Description**: Complete password reset with new password
- **Request Body**:
  ```json
  {
    "token": "reset-token-string",
    "newPassword": "NewSecurePass456!"
  }
  ```
- **Validation**:
  - Token: Required
  - NewPassword: Required, 8+ chars, mixed case, number, special char
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "message": "Password reset successfully"
  }
  ```
- **Response (Error 400)**:
  ```json
  {
    "success": false,
    "message": "Invalid or expired token"
  }
  ```

## Account Management Endpoints

### Delete Account
- **Endpoint**: `DELETE /api/auth/account`
- **Description**: Permanently delete user account and all associated data
- **Request Body**:
  ```json
  {
    "userId": "uuid-string"
  }
  ```
- **Response (Success 200)**:
  ```json
  {
    "success": true,
    "message": "Account deleted successfully"
  }
  ```
- **Response (Error 401)**:
  ```json
  {
    "success": false,
    "message": "Authentication required"
  }
  ```

## Security Features

- **Rate Limiting**: Authentication endpoints are rate-limited to 5 requests per 15 minutes per IP
- **Input Sanitization**: All inputs are sanitized to prevent injection attacks
- **Password Strength**: Enforced minimum 8 characters with mixed case, numbers, and special characters
- **Secure Storage**: Passwords are stored using bcrypt hashing
- **Token Expiration**: Password reset tokens expire after 1 hour
- **Session Management**: 30-day persistent sessions with Better Auth