import React, { useState, useEffect } from 'react';
import axios from 'axios';

interface ResetPasswordFormProps {
  token?: string;
  onSuccess?: () => void;
}

interface FormData {
  newPassword: string;
  confirmPassword: string;
}

const NEXT_PUBLIC_API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).env?.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'
  : process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'; // Backend runs on port 3002

const ResetPasswordForm: React.FC<ResetPasswordFormProps> = ({ token: propToken, onSuccess }) => {
  const [token, setToken] = useState<string | null>(null);
  const [formData, setFormData] = useState<FormData>({
    newPassword: '',
    confirmPassword: '',
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');
  const [validToken, setValidToken] = useState<boolean | null>(null);
  const [tokenChecked, setTokenChecked] = useState(false);

  // Get token from URL query param if not provided as prop
  useEffect(() => {
    if (propToken) {
      setToken(propToken);
      validateToken(propToken);
    } else {
      const urlParams = new URLSearchParams(window.location.search);
      const urlToken = urlParams.get('token');
      if (urlToken) {
        setToken(urlToken);
        validateToken(urlToken);
      } else {
        setTokenChecked(true);
        setValidToken(false);
      }
    }
  }, []);

  const validateToken = async (tokenStr: string) => {
    try {
      const response = await axios.get(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/reset-password/validate?token=${tokenStr}`);
      setValidToken(response.data.valid);
    } catch (error) {
      setValidToken(false);
    } finally {
      setTokenChecked(true);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value,
    });
    
    // Clear error when user starts typing
    if (errors[name]) {
      setErrors({
        ...errors,
        [name]: '',
      });
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setErrors({});
    setSuccessMessage('');

    // Validate passwords match
    if (formData.newPassword !== formData.confirmPassword) {
      setErrors({
        confirmPassword: 'Passwords do not match'
      });
      setLoading(false);
      return;
    }

    try {
      const response = await axios.post(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/reset-password/complete`, {
        token,
        newPassword: formData.newPassword
      });
      
      if (response.status === 200) {
        setSuccessMessage('Password reset successfully! You can now log in with your new password.');
        if (onSuccess) onSuccess();
        
        // Clear form after successful reset
        setFormData({
          newPassword: '',
          confirmPassword: '',
        });
      }
    } catch (error: any) {
      if (error.response) {
        const { status, data } = error.response;
        
        if (status === 400) {
          // Handle validation errors
          const newErrors: Record<string, string> = {};
          if (data.errors && Array.isArray(data.errors)) {
            data.errors.forEach((err: any) => {
              newErrors[err.field] = err.message;
            });
          }
          setErrors(newErrors);
        } else {
          setErrors({ general: data.message || 'An error occurred resetting the password' });
        }
      } else {
        setErrors({ general: 'Network error. Please try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  // Password strength calculation
  const getPasswordStrength = () => {
    const password = formData.newPassword;
    let strength = 0;
    
    // Length check
    if (password.length >= 8) strength += 1;
    if (password.length >= 12) strength += 1;
    
    // Character variety checks
    if (/[a-z]/.test(password)) strength += 1;
    if (/[A-Z]/.test(password)) strength += 1;
    if (/\d/.test(password)) strength += 1;
    if (/[^a-zA-Z\d]/.test(password)) strength += 1;
    
    return strength;
  };

  const getPasswordStrengthClass = () => {
    const strength = getPasswordStrength();
    if (strength <= 1) return 'weak';
    if (strength <= 3) return 'medium';
    if (strength >= 4) return 'strong';
    return '';
  };

  const getPasswordStrengthText = () => {
    const strength = getPasswordStrength();
    if (strength <= 1) return 'Weak';
    if (strength <= 3) return 'Medium';
    if (strength >= 4) return 'Strong';
    return '';
  };

  if (!tokenChecked) {
    return <div>Validating token...</div>;
  }

  if (validToken === false) {
    return (
      <div className="reset-password-form-container">
        <div className="invalid-token">
          <h2>Invalid Token</h2>
          <p>The password reset link is invalid or has expired. Please request a new reset link.</p>
        </div>
      </div>
    );
  }

  return (
    <div className="reset-password-form-container">
      <form onSubmit={handleSubmit} className="reset-password-form">
        <h2>Reset Password</h2>
        
        {errors.general && (
          <div className="error-message" style={{ color: 'red', marginBottom: '10px' }}>
            {errors.general}
          </div>
        )}
        
        {successMessage && (
          <div className="success-message" style={{ color: 'green', marginBottom: '10px' }}>
            {successMessage}
          </div>
        )}
        
        <div className="form-group">
          <label htmlFor="newPassword">New Password:</label>
          <input
            type="password"
            id="newPassword"
            name="newPassword"
            value={formData.newPassword}
            onChange={handleChange}
            required
            className={errors.newPassword ? 'error' : ''}
          />
          <div className="password-strength-indicator">
            <div className="strength-label">Password strength:</div>
            <div className="strength-bars">
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
            </div>
            <div className="strength-text">{getPasswordStrengthText()}</div>
          </div>
          {errors.newPassword && <div className="error">{errors.newPassword}</div>}
        </div>
        
        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            required
            className={errors.confirmPassword ? 'error' : ''}
          />
          {errors.confirmPassword && <div className="error">{errors.confirmPassword}</div>}
        </div>
        
        <button type="submit" disabled={loading}>
          {loading ? 'Resetting...' : 'Reset Password'}
        </button>
      </form>
      
      <style jsx>{`
        .reset-password-form-container {
          max-width: 400px;
          margin: 0 auto;
          padding: 20px;
        }
        
        .invalid-token {
          text-align: center;
          padding: 30px;
        }
        
        .invalid-token h2 {
          color: #e74c3c;
          margin-bottom: 15px;
        }
        
        .reset-password-form {
          background: #fff;
          padding: 30px;
          border-radius: 8px;
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        
        .reset-password-form h2 {
          text-align: center;
          margin-bottom: 20px;
          color: #333;
        }
        
        .form-group {
          margin-bottom: 20px;
        }
        
        .form-group label {
          display: block;
          margin-bottom: 5px;
          font-weight: bold;
          color: #555;
        }
        
        .form-group input {
          width: 100%;
          padding: 10px;
          border: 1px solid #ddd;
          border-radius: 4px;
          font-size: 16px;
        }
        
        .form-group input.error {
          border-color: #e74c3c;
        }
        
        .password-strength-indicator {
          margin-top: 8px;
        }
        
        .strength-label {
          font-size: 12px;
          color: #666;
          margin-bottom: 5px;
        }
        
        .strength-bars {
          display: flex;
          gap: 3px;
        }
        
        .strength-bar {
          flex: 1;
          height: 5px;
          background: #e0e0e0;
          border-radius: 2px;
        }
        
        .strength-bar.weak {
          background: #e74c3c;
        }
        
        .strength-bar.medium {
          background: #f39c12;
        }
        
        .strength-bar.strong {
          background: #2ecc71;
        }
        
        .strength-text {
          font-size: 12px;
          color: #666;
          margin-top: 3px;
        }
        
        .error-message {
          margin-bottom: 15px;
          padding: 10px;
          border-radius: 4px;
          background-color: #ffebee;
          border: 1px solid #ffcdd2;
        }
        
        .success-message {
          margin-bottom: 15px;
          padding: 10px;
          border-radius: 4px;
          background-color: #e8f5e9;
          border: 1px solid #c8e6c9;
        }
        
        .error {
          color: #e74c3c;
          font-size: 14px;
        }
        
        button {
          width: 100%;
          padding: 12px;
          background: #3498db;
          color: white;
          border: none;
          border-radius: 4px;
          font-size: 16px;
          cursor: pointer;
        }
        
        button:disabled {
          background: #bdc3c7;
          cursor: not-allowed;
        }
        
        button:hover:not(:disabled) {
          background: #2980b9;
        }
      `}</style>
    </div>
  );
};

export default ResetPasswordForm;