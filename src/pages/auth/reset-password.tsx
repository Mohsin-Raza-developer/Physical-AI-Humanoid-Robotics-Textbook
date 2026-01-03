import React, { useState, useEffect } from 'react';
import axios from 'axios';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface FormData {
  newPassword: string;
  confirmPassword: string;
}

const ResetPasswordPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const authBaseUrl = (siteConfig.customFields?.authBaseUrl as string) || 'https://auth-backend-mohsin-raza-developers-projects.vercel.app';
  const [formData, setFormData] = useState<FormData>({
    newPassword: '',
    confirmPassword: ''
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');
  const [validToken, setValidToken] = useState<boolean | null>(null); // null = checking, true/false = result
  const [token, setToken] = useState<string>('');
  const location = useLocation();
  const logoUrl = useBaseUrl('/img/logo.png');
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);

  // Get token from query params when component mounts
  useEffect(() => {
    const searchParams = new URLSearchParams(location.search);
    const tokenParam = searchParams.get('token');
    if (tokenParam) {
      setToken(tokenParam);
      validateToken(tokenParam);
    } else {
      setValidToken(false);
    }
  }, [location.search]);

  // Validate the token when page loads
  const validateToken = async (token: string) => {
    try {
      const response = await axios.get(`${authBaseUrl}/api/auth/reset-password/validate?token=${token}`);
      setValidToken(response.data.valid);
    } catch (error) {
      setValidToken(false);
    }
  };

  // Calculate password strength
  const getPasswordStrength = () => {
    const password = formData.newPassword;
    let strength = 0;

    if (!password) return 0;

    // Length check
    if (password.length >= 8) strength += 1;
    if (password.length >= 12) strength += 1;

    // Character variety checks
    if (/[a-z]/.test(password)) strength += 1;
    if (/[A-Z]/.test(password)) strength += 1;
    if (/\d/.test(password)) strength += 1;
    if (/[^a-zA-Z\d\s]/.test(password)) strength += 1;

    return strength;
  };

  const getPasswordStrengthClass = () => {
    const strength = getPasswordStrength();
    if (strength <= 1) return 'weak';
    if (strength <= 3) return 'medium';
    if (strength >= 4) return 'strong';
    return 'empty';
  };

  const getPasswordStrengthText = () => {
    const strength = getPasswordStrength();
    if (strength <= 1) return 'Very Weak';
    if (strength === 2) return 'Weak';
    if (strength === 3) return 'Medium';
    if (strength >= 4) return 'Strong';
    return 'Enter Password';
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

    // Validate password strength
    if (getPasswordStrength() < 3) {
      setErrors({
        newPassword: 'Please use a stronger password'
      });
      setLoading(false);
      return;
    }

    try {
      const response = await axios.post(`${authBaseUrl}/api/auth/reset-password/complete`, {
        token,
        newPassword: formData.newPassword
      });

      if (response.status === 200) {
        setSuccessMessage('Password reset successfully! You can now log in with your new password.');
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

  if (validToken === null) {
    return (
      <div className="loading-container">
        <p>Validating reset token...</p>
      </div>
    );
  }

  if (validToken === false) {
    return (
      <div className="invalid-token-container">
        <h2>Invalid Token</h2>
        <p>The password reset link is invalid or has expired. Please request a new reset link.</p>
        <Link to="/auth/forgot-password" className="auth-button">
          Request New Reset Link
        </Link>
      </div>
    );
  }

  return (
    <div className="reset-password-page">
      <div className="reset-password-card">
        <div className="card-header">
          <div className="auth-logo-container">
            <img src={logoUrl} alt="Physical AI & Humanoid Robotics Logo" className="auth-logo" />
          </div>
          <h2>Reset Your Password</h2>
          <p className="form-subtitle">Enter your new password.</p>
        </div>

        {errors.general && (
          <div className="error-toast">
            <div className="error-toast-icon">⚠️</div>
            <div className="error-toast-message">{errors.general}</div>
          </div>
        )}

        {successMessage ? (
          <div className="success-toast">
            <div className="success-toast-icon">✅</div>
            <div className="success-toast-message">{successMessage}</div>
            <Link to="/auth/login" className="auth-button mt-3">
              Go to Login
            </Link>
          </div>
        ) : (
          <form onSubmit={handleSubmit} className="reset-password-form">
            <div className="form-group">
              <div className="input-wrapper">
                <div className="input-icon">🔒</div>
                <input
                  type={showPassword ? "text" : "password"}
                  id="newPassword"
                  name="newPassword"
                  value={formData.newPassword}
                  onChange={handleChange}
                  required
                  placeholder="Enter your new password"
                  className={errors.newPassword ? 'error' : ''}
                  aria-describedby={errors.newPassword ? "newPassword-error" : undefined}
                />
                <button
                  type="button"
                  className="password-toggle"
                  onClick={() => setShowPassword(!showPassword)}
                  tabIndex={-1}
                >
                  {showPassword ? (
                    <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" /></svg>
                  ) : (
                    <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" /><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" /></svg>
                  )}
                </button>
              </div>
              <div className="password-strength-container">
                <div className="strength-label">Password strength:</div>
                <div className="strength-bars">
                  <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
                  <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
                  <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
                  <div className={`strength-bar ${getPasswordStrengthClass()}`}></div>
                </div>
                <div className="strength-text">{getPasswordStrengthText()}</div>
              </div>
              {errors.newPassword && (
                <div id="newPassword-error" className="field-error">
                  {errors.newPassword}
                </div>
              )}
            </div>

            <div className="form-group">
              <div className="input-wrapper">
                <div className="input-icon">🔒</div>
                <input
                  type={showConfirmPassword ? "text" : "password"}
                  id="confirmPassword"
                  name="confirmPassword"
                  value={formData.confirmPassword}
                  onChange={handleChange}
                  required
                  placeholder="Confirm your new password"
                  className={errors.confirmPassword ? 'error' : ''}
                  aria-describedby={errors.confirmPassword ? "confirmPassword-error" : undefined}
                />
                <button
                  type="button"
                  className="password-toggle"
                  onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                  tabIndex={-1}
                >
                  {showConfirmPassword ? (
                    <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" /></svg>
                  ) : (
                    <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" /><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" /></svg>
                  )}
                </button>
              </div>
              {errors.confirmPassword && (
                <div id="confirmPassword-error" className="field-error">
                  {errors.confirmPassword}
                </div>
              )}
            </div>

            <button type="submit" disabled={loading} className="auth-button">
              {loading ? (
                <span className="loading-spinner">
                  <span className="spinner"></span> Resetting Password...
                </span>
              ) : (
                'Reset Password'
              )}
            </button>
          </form>
        )}
      </div>

      <style jsx>{`
        /* Premium Glassmorphism Design */
        .reset-password-page {
          width: 100%;
          min-height: 100vh;
          display: flex;
          justify-content: center;
          align-items: center;
          padding: 20px;
          background: radial-gradient(circle at top right, rgba(var(--ifm-color-primary-rgb), 0.1), transparent 40%),
                      radial-gradient(circle at bottom left, rgba(var(--ifm-color-primary-rgb), 0.05), transparent 40%);
        }

        .reset-password-card {
          width: 100%;
          max-width: 450px;
          background: var(--ifm-card-background-color);
          backdrop-filter: blur(20px);
          -webkit-backdrop-filter: blur(20px);
          border-radius: 20px;
          padding: 40px;
          box-shadow: 0 20px 80px rgba(21, 223, 230, 0.08);
          border: 1px solid var(--ifm-color-emphasis-200);
          position: relative;
          overflow: hidden;
        }

        .card-header {
          text-align: center;
          margin-bottom: 30px;
        }

        .card-header h2 {
          margin: 15px 0 10px 0;
          font-size: 2rem;
          font-weight: 700;
          background: linear-gradient(135deg, var(--ifm-color-primary-darker) 0%, var(--ifm-color-primary) 100%);
          -webkit-background-clip: text;
          -webkit-text-fill-color: transparent;
        }

        .form-subtitle {
          color: var(--ifm-color-emphasis-600);
          font-size: 1.1rem;
        }

        .auth-logo-container {
          display: flex;
          justify-content: center;
          margin-bottom: 0px;
        }

        .auth-logo {
          height: 80px;
          width: auto;
          filter: drop-shadow(0 4px 6px rgba(0,0,0,0.1));
        }

        .form-group {
          margin-bottom: 24px;
        }

        .input-wrapper {
          position: relative;
          display: flex;
          align-items: center;
        }

        .input-icon {
          position: absolute;
          left: 16px;
          top: 50%;
          transform: translateY(-50%);
          color: var(--ifm-color-primary);
          font-size: 1.2rem;
          z-index: 2;
          pointer-events: none;
          opacity: 0.7;
          transition: opacity 0.3s;
        }

        .input-wrapper:focus-within .input-icon {
          opacity: 1;
        }

        .password-toggle {
          position: absolute;
          right: 12px;
          top: 50%;
          transform: translateY(-50%);
          background: none;
          border: none;
          padding: 4px;
          cursor: pointer;
          color: var(--ifm-color-emphasis-500);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: color 0.2s;
          border-radius: 50%;
          z-index: 5;
        }

        .password-toggle:hover {
          color: var(--ifm-color-primary);
          background-color: rgba(0,0,0,0.05);
        }

        input {
          width: 100%;
          padding: 16px 16px 16px 50px;
          border: 2px solid transparent;
          border-radius: 12px;
          font-size: 1rem;
          transition: all 0.3s ease;
          background-color: var(--ifm-color-emphasis-100);
          color: var(--ifm-font-color-base);
          box-shadow: inset 0 2px 4px rgba(0,0,0,0.02);
        }

        input:hover {
          background-color: var(--ifm-color-emphasis-200);
        }

        input:focus {
          outline: none;
          background-color: var(--ifm-background-color);
          border-color: var(--ifm-color-primary);
          box-shadow: 0 0 0 4px rgba(var(--ifm-color-primary-rgb), 0.15);
        }

        input.error {
          border-color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-lightest);
        }

        /* Strength Meter */
        .password-strength-container {
          margin-top: 12px;
          background: var(--ifm-color-emphasis-100);
          padding: 10px;
          border-radius: 8px;
        }

        .strength-label {
          font-size: 0.8rem;
          color: var(--ifm-color-emphasis-600);
          margin-bottom: 6px;
          display: flex;
          justify-content: space-between;
        }

        .strength-bars {
          display: flex;
          gap: 4px;
          height: 6px;
        }

        .strength-bar {
          flex: 1;
          height: 100%;
          background: var(--ifm-color-emphasis-300);
          border-radius: 3px;
          transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .strength-bar.weak { background: #ff4d4f; }
        .strength-bar.medium { background: #faad14; }
        .strength-bar.strong { background: #52c41a; }

        .strength-text {
          font-size: 0.8rem;
          text-align: right;
          margin-top: 4px;
          font-weight: 600;
        }

        /* Error Messages */
        .field-error {
          color: var(--ifm-color-danger);
          font-size: 0.85rem;
          margin-top: 8px;
          display: flex;
          align-items: center;
          gap: 6px;
        }

        /* Buttons */
        .auth-button {
          width: 100%;
          padding: 16px;
          background: linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--ifm-color-primary-darker) 100%);
          color: white;
          border: none;
          border-radius: 12px;
          font-size: 1.1rem;
          font-weight: 700;
          cursor: pointer;
          transition: all 0.3s ease;
          box-shadow: 0 10px 20px rgba(var(--ifm-color-primary-rgb), 0.2);
          text-align: center;
          text-decoration: none;
          display: inline-block;
          margin-top: 10px;
        }

        .auth-button:hover:not(:disabled) {
          transform: translateY(-3px);
          box-shadow: 0 15px 30px rgba(var(--ifm-color-primary-rgb), 0.3);
        }

        .auth-button:disabled {
          background: var(--ifm-color-emphasis-300);
          color: var(--ifm-color-emphasis-500);
          cursor: not-allowed;
          box-shadow: none;
          transform: none;
        }

        .loading-spinner, .spinner { /* Keep existing spinner styles if needed */ }

        @media (max-width: 480px) {
          .reset-password-card { padding: 24px; border-radius: 16px; }
        }
`}</style>
    </div>
  );
};

export default ResetPasswordPage;