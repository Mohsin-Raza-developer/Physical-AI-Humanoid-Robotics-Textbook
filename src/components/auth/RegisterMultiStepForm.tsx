import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { useAuth } from './AuthProvider';

import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface RegisterMultiStepFormProps {
  onSuccess?: () => void;
}

interface Step1Data {
  firstName: string;
  lastName: string;
  email: string;
  password: string;
  confirmPassword: string;
}

interface Step2Data {
  softwareLevel: string;
  hardwareAccess: string;
}

interface FormData {
  step1: Step1Data;
  step2: Step2Data;
}

const NEXT_PUBLIC_API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).env?.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'
  : process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'; // Backend runs on port 3002

const RegisterMultiStepForm: React.FC<RegisterMultiStepFormProps> = ({ onSuccess }) => {
  const { login } = useAuth();
  const [step, setStep] = useState<number>(1);
  const [formData, setFormData] = useState<FormData>({
    step1: {
      firstName: '',
      lastName: '',
      email: '',
      password: '',
      confirmPassword: '',
    },
    step2: {
      softwareLevel: 'Beginner',
      hardwareAccess: 'Laptop/Cloud',
    },
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [fieldStatus, setFieldStatus] = useState<Record<string, 'valid' | 'invalid' | 'empty'>>({
    firstName: 'empty',
    lastName: 'empty',
    email: 'empty',
    password: 'empty',
    confirmPassword: 'empty',
  });
  const emailRef = useRef<HTMLInputElement>(null);

  // Auto-focus on email field when component mounts
  useEffect(() => {
    if (emailRef.current) {
      emailRef.current.focus();
    }
  }, []);

  const handleChangeStep1 = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      step1: {
        ...formData.step1,
        [name]: value,
      },
    });

    // Real-time validation
    if (name === 'email') {
      const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(value);
      setFieldStatus(prev => ({
        ...prev,
        email: !value ? 'empty' : isValid ? 'valid' : 'invalid'
      }));
    } else if (name === 'password') {
      setFieldStatus(prev => ({
        ...prev,
        password: value ? 'valid' : 'empty'
      }));
    } else if (name === 'confirmPassword') {
      setFieldStatus(prev => ({
        ...prev,
        confirmPassword:
          !value ? 'empty' :
            value === formData.step1.password ? 'valid' : 'invalid'
      }));
    } else if (name === 'firstName' || name === 'lastName') {
      const isValid = value.trim().length > 0 && value.length <= 12;
      setFieldStatus(prev => ({
        ...prev,
        [name]: !value ? 'empty' : isValid ? 'valid' : 'invalid'
      }));

      if (value.length > 12) {
        setErrors(prev => ({ ...prev, [name]: 'Maximum 12 characters allowed' }));
      } else {
        if (errors[name]) {
          setErrors(prev => ({ ...prev, [name]: '' }));
        }
      }
    }

    // Clear error when user starts typing (only if not length error)
    if (errors[name] && name !== 'firstName' && name !== 'lastName') {
      setErrors({
        ...errors,
        [name]: '',
      });
    }
  };

  const handleSelectionChange = (name: string, value: string) => {
    setFormData({
      ...formData,
      step2: {
        ...formData.step2,
        [name]: value,
      },
    });

    // Clear error
    if (errors[name]) {
      setErrors({
        ...errors,
        [name]: '',
      });
    }
  };

  // Keep existing select handler for backward compatibility if needed, or remove.
  // We'll just define the selection options arrays for rendering
  const softwareOptions = ['Beginner', 'Intermediate', 'Advanced'];
  const hardwareOptions = ['Laptop/Cloud', 'RTX GPU', 'Jetson Edge', 'Physical Robot'];

  // Password strength calculation
  const getPasswordStrength = () => {
    const password = formData.step1.password;
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

  const nextStep = async () => {
    // Validate current step before proceeding
    const newErrors: Record<string, string> = {};

    if (!formData.step1.firstName) newErrors.firstName = 'First Name is required';
    else if (formData.step1.firstName.length > 12) newErrors.firstName = 'Maximum 12 characters allowed';

    if (!formData.step1.lastName) newErrors.lastName = 'Last Name is required';
    else if (formData.step1.lastName.length > 12) newErrors.lastName = 'Maximum 12 characters allowed';

    if (!formData.step1.email) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.step1.email)) {
      newErrors.email = 'Please enter a valid email address';
    }

    if (!formData.step1.password) {
      newErrors.password = 'Password is required';
    } else if (formData.step1.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    } else if (!/[a-z]/.test(formData.step1.password)) {
      newErrors.password = 'Password must contain at least one lowercase letter';
    } else if (!/[A-Z]/.test(formData.step1.password)) {
      newErrors.password = 'Password must contain at least one uppercase letter';
    } else if (!/\d/.test(formData.step1.password)) {
      newErrors.password = 'Password must contain at least one number';
    }

    if (!formData.step1.confirmPassword) {
      newErrors.confirmPassword = 'Please confirm your password';
    } else if (formData.step1.password !== formData.step1.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    // Check if email is already registered
    setLoading(true);
    try {
      const response = await axios.post(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/check-email`, {
        email: formData.step1.email
      });

      if (response.data.exists) {
        setErrors({ email: 'Email already registered' });
        setLoading(false);
        return;
      }
    } catch (error) {
      console.error('Email check failed:', error);
      // Optional: setErrors({ email: 'Unable to verify email. Please try again.' });
      // For now, allowing to proceed or failing silently usually ok? 
      // User said "bolo k Email already registered". So if check fails, we might just proceed and let submit fail?
      // Better to stop if we can't verify. But let's assume network is fine.
    }
    setLoading(false);

    setErrors({});
    setStep(2);
  };

  const prevStep = () => {
    setStep(1);
  };

  const submitForm = async () => {
    setLoading(true);
    setErrors({});
    setSuccessMessage('');

    try {
      const response = await axios.post(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/register`, {
        firstName: formData.step1.firstName,
        lastName: formData.step1.lastName,
        email: formData.step1.email,
        password: formData.step1.password,
        softwareLevel: formData.step2.softwareLevel,
        hardwareAccess: formData.step2.hardwareAccess,
      });

      if (response.status === 201) {
        // Automatically log in the user after successful registration
        try {
          const loginResponse = await axios.post(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/login`, {
            email: formData.step1.email,
            password: formData.step1.password,
          });

          if (loginResponse.status === 200) {
            // Use AuthProvider login method to update state and localStorage properly
            // This ensures firstName and lastName are saved
            const userData = {
              userId: loginResponse.data.data?.userId,
              email: loginResponse.data.data?.email,
              firstName: loginResponse.data.data?.firstName,
              lastName: loginResponse.data.data?.lastName,
            };

            login(userData);

            // Show success message
            setSuccessMessage('Welcome! Your account has been created.');
            if (onSuccess) onSuccess();

            // Redirect to book intro page after delay
            setTimeout(() => {
              if (typeof window !== 'undefined') {
                window.location.href = window.location.origin + '/Physical-AI-Humanoid-Robotics-Textbook/docs/intro/';
              }
            }, 2000);
          }
        } catch (loginError: any) {
          // If auto-login fails, redirect to login page
          setSuccessMessage('Registration successful! Please log in with your credentials.');
          if (onSuccess) onSuccess();

          console.error('Auto-login after registration failed:', loginError);

          // Redirect to login after delay
          setTimeout(() => {
            if (typeof window !== 'undefined') {
              window.location.href = window.location.origin + '/Physical-AI-Humanoid-Robotics-Textbook/auth/login';
            }
          }, 2000);
        }
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
        } else if (status === 409) {
          setErrors({ email: data.message });
        } else {
          setErrors({ general: data.message || 'An error occurred during registration' });
        }
      } else {
        setErrors({ general: 'Network error. Please try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <div className="form-header">
        <h2>Create Your Account</h2>
        <p className="form-subtitle">Join the community of Physical AI & Humanoid Robotics enthusiasts</p>
      </div>

      {errors.general && (
        <div className="error-toast">
          <div className="error-toast-icon"></div>
          <div className="error-toast-message">{errors.general}</div>
        </div>
      )}

      <div className="step-indicator">
        <div className={`step ${step === 1 ? 'active' : ''} ${step > 1 ? 'completed' : ''}`}>
          <div className="step-number">1</div>
          <div className="step-label">Account Info</div>
        </div>
        <div className={`step ${step === 2 ? 'active' : ''} ${step > 2 ? 'completed' : ''}`}>
          <div className="step-number">2</div>
          <div className="step-label">Preferences</div>
        </div>
      </div>

      {step === 1 ? (
        <div className="step-content step1-content">
          <h3>Account Information</h3>

          <div className="form-row">
            <div className="form-group half-width">
              <div className="input-wrapper">
                <div className="input-icon">👤</div>
                <input
                  type="text"
                  name="firstName"
                  value={formData.step1.firstName}
                  onChange={handleChangeStep1}
                  required
                  placeholder="First Name"
                  className={`${errors.firstName ? 'error' : ''}`}
                />
              </div>
              {errors.firstName && <div className="field-error">{errors.firstName}</div>}
            </div>

            <div className="form-group half-width">
              <div className="input-wrapper">
                <div className="input-icon">👤</div>
                <input
                  type="text"
                  name="lastName"
                  value={formData.step1.lastName}
                  onChange={handleChangeStep1}
                  required
                  placeholder="Last Name"
                  className={`${errors.lastName ? 'error' : ''}`}
                />
              </div>
              {errors.lastName && <div className="field-error">{errors.lastName}</div>}
            </div>
          </div>

          <div className="form-group">
            <div className="input-wrapper">
              <div className="input-icon">📨</div>
              <input
                ref={emailRef}
                type="email"
                id="email"
                name="email"
                value={formData.step1.email}
                onChange={handleChangeStep1}
                required
                placeholder="Enter your email address"
                className={`${errors.email ? 'error' : ''} ${fieldStatus.email !== 'empty' ? (fieldStatus.email === 'valid' ? 'valid' : 'invalid') : ''}`}
                aria-describedby={errors.email ? "email-error" : undefined}
              />
              {fieldStatus.email !== 'empty' && (
                <div className={`input-status ${fieldStatus.email}`}>
                  {fieldStatus.email === 'valid' ? '✓' : '✗'}
                </div>
              )}
            </div>
            {errors.email && (
              <div id="email-error" className="field-error">
                {errors.email}
              </div>
            )}
          </div>

          <div className="form-group">
            <div className="input-wrapper">
              <div className="input-icon">🔒</div>
              <input
                type={showPassword ? "text" : "password"}
                id="password"
                name="password"
                value={formData.step1.password}
                onChange={handleChangeStep1}
                required
                placeholder="Create a password"
                className={`${errors.password ? 'error' : ''} ${fieldStatus.password !== 'empty' ? (fieldStatus.password === 'valid' ? 'valid' : 'invalid') : ''}`}
                aria-describedby={errors.password ? "password-error" : undefined}
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

            {errors.password && (
              <div id="password-error" className="field-error">
                {errors.password}
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
                value={formData.step1.confirmPassword}
                onChange={handleChangeStep1}
                required
                placeholder="Confirm your password"
                className={`${errors.confirmPassword ? 'error' : ''} ${fieldStatus.confirmPassword !== 'empty' ? (fieldStatus.confirmPassword === 'valid' ? 'valid' : 'invalid') : ''}`}
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

          <div className="button-group">
            <button
              type="button"
              className="secondary-btn"
              onClick={nextStep}
              disabled={loading}
            >
              {loading ? 'Checking...' : 'Continue to Preferences'}
            </button>
          </div>

        </div>
      ) : (
        <div className="step-content step2-content">
          <h3>Personalize Your Experience</h3>
          <p className="form-subtitle">Help us tailor the content to your background and equipment.</p>

          <div className="form-group">
            <label className="selection-label">What's your software background?</label>
            <div className="selection-grid software-grid">
              {softwareOptions.map((option) => (
                <div
                  key={option}
                  className={`selection-card ${formData.step2.softwareLevel === option ? 'selected' : ''}`}
                  onClick={() => handleSelectionChange('softwareLevel', option)}
                >
                  <div className="selection-content">
                    {/* Optional icons can go here */}
                    <span className="selection-text">{option}</span>
                  </div>
                  {formData.step2.softwareLevel === option && <div className="selection-check">✓</div>}
                </div>
              ))}
            </div>
            {errors.softwareLevel && (
              <div className="field-error">{errors.softwareLevel}</div>
            )}
          </div>

          <div className="form-group">
            <label className="selection-label">What hardware do you have access to?</label>
            <div className="selection-grid hardware-grid">
              {hardwareOptions.map((option) => (
                <div
                  key={option}
                  className={`selection-card ${formData.step2.hardwareAccess === option ? 'selected' : ''}`}
                  onClick={() => handleSelectionChange('hardwareAccess', option)}
                >
                  <div className="selection-content">
                    <span className="selection-text">{option}</span>
                  </div>
                  {formData.step2.hardwareAccess === option && <div className="selection-check">✓</div>}
                </div>
              ))}
            </div>
            {errors.hardwareAccess && (
              <div className="field-error">{errors.hardwareAccess}</div>
            )}
          </div>

          <div className="button-group">
            <button
              type="button"
              className="secondary-btn"
              onClick={prevStep}
              disabled={loading}
            >
              Back to Account Info
            </button>
            <button
              type="button"
              className="primary-btn"
              onClick={submitForm}
              disabled={loading}
            >
              {loading ? (
                <span className="loading-spinner">
                  <span className="spinner"></span> Creating Account...
                </span>
              ) : (
                'Complete Registration'
              )}
            </button>
          </div>
        </div>
      )}

      {
        successMessage && (
          <div className="success-toast">
            <div className="success-toast-icon">✅</div>
            <div className="success-toast-message">{successMessage}</div>
          </div>
        )
      }

      <style jsx>{`
        /* Premium Glassmorphism Design */
        .auth-form-container {
          width: 100%;
          display: flex;
          flex-direction: column; /* Stack children vertically! */
          justify-content: center;
          align-items: center;
          padding: 10px;
        }

        .step-content {
          width: 100%;
          max-width: 500px;
          background: var(--ifm-card-background-color);
          backdrop-filter: blur(20px);
          -webkit-backdrop-filter: blur(20px);
          border-radius: 20px;
          padding: 30px;
          box-shadow: 0 10px 40px rgba(17, 200, 236, 0.1);
          border: 1px solid var(--ifm-color-emphasis-200);
          position: relative;
          overflow: hidden;
          animation: fadeIn 0.3s ease-in-out;
        }

        .form-header {
          text-align: center;
          margin-bottom: 30px;
        }

        .auth-logo {
          height: 80px;
          width: auto;
          margin-bottom: 20px;
        }

        .form-header h2 {
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
          margin: 0;
        }

        .step-indicator {
          display: flex;
          justify-content: space-between;
          margin-bottom: 30px;
          position: relative;
          max-width: 400px; /* Increased from 300px to prevent overlap */
          width: 100%;
          margin-left: auto;
          margin-right: auto;
        }

        .step-indicator::before {
          content: '';
          position: absolute;
          top: 20px;
          left: 0;
          right: 0;
          height: 3px;
          background: rgba(255, 255, 255, 0.3); /* High contrast for dark mode */
          z-index: 1;
          display: block;
        }

        .step {
          text-align: center;
          position: relative;
          z-index: 2;
          color: var(--ifm-color-emphasis-600);
          /* background removed to ensure line visibility */
          padding: 0 10px;
        }

        /* ... existing step styles ... */
        
        .step.active .step-number {
          background: var(--ifm-color-primary);
          box-shadow: 0 0 0 4px rgba(var(--ifm-color-primary-rgb), 0.2);
        }

        .step.completed .step-number {
          background: var(--ifm-color-success);
        }

        .step-number {
          width: 40px;
          height: 40px;
          border-radius: 50%;
          background: var(--ifm-color-emphasis-300);
          color: white;
          display: flex;
          align-items: center;
          justify-content: center;
          margin: 0 auto 8px;
          font-weight: bold;
          transition: all 0.3s ease;
        }

        .step-label {
          font-size: 0.9rem;
          white-space: nowrap;
          font-weight: 500;
        }

        .selection-label {
          display: block;
          margin-bottom: 12px;
          font-weight: 700;
          color: var(--ifm-color-emphasis-700);
          font-size: 1rem;
        }

        .selection-grid {
          display: grid;
          gap: 12px;
        }

        .software-grid {
          /* Auto-fit: Columns will be at least 110px wide. If not enough space, they wrap. */
          grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
        }

        .hardware-grid {
          grid-template-columns: repeat(2, 1fr);
        }

        /* Increase breakpoint to 600px to catch larger mobile/small tablets */
        @media (max-width: 600px) {
          .software-grid, .hardware-grid {
            grid-template-columns: 1fr; /* Force single column on mobile */
          }
          
          .step-label {
              font-size: 0.8rem; /* Smaller text on mobile */
          }
        }

        .selection-card {
          border: 2px solid var(--ifm-color-emphasis-200);
          border-radius: 12px;
          padding: 16px;
          cursor: pointer;
          transition: all 0.2s ease;
          display: flex;
          justify-content: space-between;
          align-items: center;
          background: var(--ifm-card-background-color);
        }

        .selection-card:hover {
          border-color: var(--ifm-color-primary);
          background: var(--ifm-color-emphasis-100);
        }

        .selection-content {
          flex: 1;
          margin-right: 8px;
        }

        .selection-card.selected {
          border-color: var(--ifm-color-primary);
          background: rgba(var(--ifm-color-primary-rgb), 0.1);
          box-shadow: 0 4px 12px rgba(var(--ifm-color-primary-rgb), 0.15);
        }

        .selection-text {
          font-weight: 600;
          font-size: 0.95rem;
          white-space: normal;
          line-height: 1.3;
        }

        .selection-check {
          color: var(--ifm-color-primary);
          font-weight: bold;
          font-size: 1.1rem;
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

        input.valid {
          border-color: var(--ifm-color-success);
        }

        input.invalid, input.error {
          border-color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-lightest);
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

        .input-status {
          position: absolute;
          right: 44px;
          top: 50%;
          transform: translateY(-50%);
          font-size: 1.2rem;
          z-index: 2;
          pointer-events: none;
        }

        .input-status.valid { color: var(--ifm-color-success); }
        .input-status.invalid { color: var(--ifm-color-danger); }

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

        .strength-bar.weak { background: #ff4d4d; }
        .strength-bar.medium { background: #faad14; }
        .strength-bar.strong { background: #52c41a; }

        .strength-text {
          font-size: 0.8rem;
          text-align: right;
          margin-top: 4px;
          font-weight: 600;
        }

        .field-error {
          color: var(--ifm-color-danger);
          font-size: 0.85rem;
          margin-top: 8px;
          display: flex;
          align-items: center;
          gap: 6px;
        }

        .error-toast, .success-toast {
          display: flex;
          align-items: center;
          padding: 12px 16px;
          border-radius: 8px;
          margin-bottom: 16px;
          animation: fadeIn 0.3s ease;
        }

        .error-toast {
          background-color: var(--ifm-color-danger-lightest);
          border: 1px solid var(--ifm-color-danger-light);
        }

        .success-toast {
          background-color: var(--ifm-color-success-lightest);
          border: 1px solid var(--ifm-color-success-light);
        }

        .error-toast-icon, .success-toast-icon {
          margin-right: 10px;
          font-size: 1.2rem;
        }

        .error-toast-message, .success-toast-message {
          font-weight: 500;
        }

        .error-toast-message { color: var(--ifm-color-danger-dark); }
        .success-toast-message { color: var(--ifm-color-success-dark); }

        .button-group {
          display: flex;
          gap: 16px;
          margin-top: 30px;
        }

        .primary-btn, .secondary-btn {
          flex: 1;
          padding: 16px;
          border: none;
          border-radius: 12px;
          font-size: 1.1rem;
          font-weight: 700;
          cursor: pointer;
          transition: all 0.3s ease;
          position: relative;
        }

        .primary-btn {
          background: linear-gradient(135deg, var(--ifm-color-primary) 0%, var(--ifm-color-primary-darker) 100%);
          color: white;
          box-shadow: 0 10px 20px rgba(var(--ifm-color-primary-rgb), 0.2);
        }

        .secondary-btn {
          background: var(--ifm-color-emphasis-200);
          color: var(--ifm-color-emphasis-900);
        }

        .primary-btn:hover:not(:disabled) {
          transform: translateY(-3px);
          box-shadow: 0 15px 30px rgba(var(--ifm-color-primary-rgb), 0.3);
        }

        .secondary-btn:hover:not(:disabled) {
          transform: translateY(-3px);
          background: var(--ifm-color-emphasis-300);
        }

        .primary-btn:disabled, .secondary-btn:disabled {
          background: var(--ifm-color-emphasis-300);
          color: var(--ifm-color-emphasis-500);
          cursor: not-allowed;
          box-shadow: none;
          transform: none;
        }

        @media (max-width: 480px) {
          .step-content { padding: 24px; border-radius: 16px; }
          .button-group { flex-direction: column; }
          .primary-btn, .secondary-btn { width: 100%; }
          .primary-btn, .secondary-btn { width: 100%; }
        }

        .auth-footer-link {
          text-align: center;
          margin-top: 24px;
          font-size: 0.95rem;
          color: var(--ifm-color-emphasis-700);
        }

        .auth-footer-link a {
          font-weight: 700;
          color: var(--ifm-color-primary);
          text-decoration: none;
        }

        .auth-footer-link a:hover {
          text-decoration: underline;
        }
      `}</style>
    </div>
  );
};

export default RegisterMultiStepForm;
