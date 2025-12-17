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
    return <div className="text-center p-8">Validating token...</div>;
  }

  if (validToken === false) {
    return (
      <div className="auth-form-container">
        <div className="invalid-token text-center">
          <h2 className="text-2xl font-bold mb-4 text-red-500">Invalid Token</h2>
          <p>The password reset link is invalid or has expired. Please request a new reset link.</p>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-form-container">
      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-header">
          <h2>Reset Password</h2>
        </div>

        {errors.general && (
          <div className="error-toast">
            <div className="error-toast-icon">⚠️</div>
            <div className="error-toast-message">{errors.general}</div>
          </div>
        )}

        {successMessage && (
          <div className="success-toast">
            <div className="success-toast-icon">✅</div>
            <div className="success-toast-message">{successMessage}</div>
          </div>
        )}

        <div className="form-group">
          <label htmlFor="newPassword">New Password:</label>
          <div className="input-wrapper">
            <div className="input-icon">🔒</div>
            <input
              type="password"
              id="newPassword"
              name="newPassword"
              value={formData.newPassword}
              onChange={handleChange}
              required
              className={errors.newPassword ? 'error' : ''}
              placeholder="New Password"
            />
          </div>

          <div className="password-strength-container text-right">
            {/* Note: Ideally use logical strength meter here too */}
            <div className="strength-bars">
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
              <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
            </div>
            <div className="strength-text">{getPasswordStrengthText()}</div>
          </div>

          {errors.newPassword && <div className="field-error">{errors.newPassword}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <div className="input-wrapper">
            <div className="input-icon">🔒</div>
            <input
              type="password"
              id="confirmPassword"
              name="confirmPassword"
              value={formData.confirmPassword}
              onChange={handleChange}
              required
              className={errors.confirmPassword ? 'error' : ''}
              placeholder="Confirm Password"
            />
          </div>
          {errors.confirmPassword && <div className="field-error">{errors.confirmPassword}</div>}
        </div>

        <button type="submit" disabled={loading} className="primary-btn w-full margin-top--md">
          {loading ? 'Resetting...' : 'Reset Password'}
        </button>
      </form>
    </div>
  );
};

export default ResetPasswordForm;