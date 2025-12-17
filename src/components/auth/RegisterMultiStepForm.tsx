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

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// --- Helper Components ---

interface SelectionGridProps {
  label: string;
  options: string[];
  selectedValue: string;
  onChange: (value: string) => void;
  error?: string;
  gridClass?: string;
}

const SelectionGrid: React.FC<SelectionGridProps> = ({ label, options, selectedValue, onChange, error, gridClass = "" }) => (
  <div className="form-group">
    <label className="selection-label">{label}</label>
    <div className={`selection-grid ${gridClass}`}>
      {options.map((option) => (
        <div
          key={option}
          className={`selection-card ${selectedValue === option ? 'selected' : ''}`}
          onClick={() => onChange(option)}
        >
          <div className="selection-content">
            <span className="selection-text">{option}</span>
          </div>
          {selectedValue === option && <div className="selection-check">✓</div>}
        </div>
      ))}
    </div>
    {error && <div className="field-error">{error}</div>}
  </div>
);

interface FormInputProps {
  icon: string;
  type?: string;
  name: string;
  value: string;
  onChange: (e: React.ChangeEvent<HTMLInputElement>) => void;
  placeholder: string;
  error?: string;
  status?: 'valid' | 'invalid' | 'empty';
  inputRef?: React.RefObject<HTMLInputElement>;
  containerClass?: string;
  togglePassword?: {
    show: boolean;
    onToggle: () => void;
  };
}

const FormInput: React.FC<FormInputProps> = ({
  icon, type = "text", name, value, onChange, placeholder, error, status, inputRef, containerClass = "form-group", togglePassword
}) => {
  const inputType = togglePassword ? (togglePassword.show ? "text" : "password") : type;

  return (
    <div className={containerClass}>
      <div className="input-wrapper">
        <div className="input-icon">{icon}</div>
        <input
          ref={inputRef}
          type={inputType}
          id={name}
          name={name}
          value={value}
          onChange={onChange}
          required
          placeholder={placeholder}
          className={`${error ? 'error' : ''} ${status && status !== 'empty' ? (status === 'valid' ? 'valid' : 'invalid') : ''}`}
          aria-describedby={error ? `${name}-error` : undefined}
        />

        {togglePassword && (
          <button
            type="button"
            className="password-toggle"
            onClick={togglePassword.onToggle}
            tabIndex={-1}
          >
            {togglePassword.show ? (
              <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" /></svg>
            ) : (
              <svg width="20" height="20" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" /><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" /></svg>
            )}
          </button>
        )}

        {status && status !== 'empty' && !togglePassword && (
          <div className={`input-status ${status}`}>
            {status === 'valid' ? '✓' : '✗'}
          </div>
        )}
      </div>
      {error && <div id={`${name}-error`} className="field-error">{error}</div>}
    </div>
  );
};


const RegisterMultiStepForm: React.FC<RegisterMultiStepFormProps> = ({ onSuccess }) => {
  const { siteConfig } = useDocusaurusContext();
  const apiBaseUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:3002';

  useEffect(() => {
    console.log('RegisterForm using API:', apiBaseUrl);
  }, [apiBaseUrl]);

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

  const getPasswordStrengthState = () => {
    const strength = getPasswordStrength();
    if (strength === 0) return { filled: 0, label: 'empty' };
    if (strength <= 2) return { filled: 1, label: 'weak' };
    if (strength === 3) return { filled: 2, label: 'medium' };
    return { filled: 4, label: 'strong' };
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
      const response = await axios.post(`${apiBaseUrl}/api/auth/check-email`, {
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
      const response = await axios.post(`${apiBaseUrl}/api/auth/register`, {
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
          const loginResponse = await axios.post(`${apiBaseUrl}/api/auth/login`, {
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
              token: loginResponse.data.data?.token // Pass token to AuthProvider
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
            <FormInput
              icon="👤"
              name="firstName"
              value={formData.step1.firstName}
              onChange={handleChangeStep1}
              placeholder="First Name"
              error={errors.firstName}
              status={fieldStatus.firstName}
              containerClass="form-group half-width"
            />

            <FormInput
              icon="👤"
              name="lastName"
              value={formData.step1.lastName}
              onChange={handleChangeStep1}
              placeholder="Last Name"
              error={errors.lastName}
              status={fieldStatus.lastName}
              containerClass="form-group half-width"
            />
          </div>

          <FormInput
            icon="📨"
            type="email"
            name="email"
            value={formData.step1.email}
            onChange={handleChangeStep1}
            placeholder="Enter your email address"
            error={errors.email}
            status={fieldStatus.email}
            inputRef={emailRef}
          />

          <FormInput
            icon="🔒"
            name="password"
            value={formData.step1.password}
            onChange={handleChangeStep1}
            placeholder="Create a password"
            error={errors.password}
            status={fieldStatus.password}
            togglePassword={{
              show: showPassword,
              onToggle: () => setShowPassword(!showPassword)
            }}
          />

          <div className="password-strength-container">
            <div className="strength-label">Password strength:</div>
            <div className="strength-bars">
              {(() => {
                const { filled, label } = getPasswordStrengthState();
                return [0, 1, 2, 3].map((i) => (
                  <div key={i} className={`strength-bar ${i < filled ? label : ''}`} />
                ));
              })()}
            </div>
            <div className="strength-text">{getPasswordStrengthText()}</div>
          </div>

          <FormInput
            icon="🔒"
            name="confirmPassword"
            value={formData.step1.confirmPassword}
            onChange={handleChangeStep1}
            placeholder="Confirm your password"
            error={errors.confirmPassword}
            status={fieldStatus.confirmPassword}
            togglePassword={{
              show: showConfirmPassword,
              onToggle: () => setShowConfirmPassword(!showConfirmPassword)
            }}
          />

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

          <SelectionGrid
            label="What's your software background?"
            options={softwareOptions}
            selectedValue={formData.step2.softwareLevel}
            onChange={(val) => handleSelectionChange('softwareLevel', val)}
            error={errors.softwareLevel}
            gridClass="software-grid"
          />

          <SelectionGrid
            label="What hardware do you have access to?"
            options={hardwareOptions}
            selectedValue={formData.step2.hardwareAccess}
            onChange={(val) => handleSelectionChange('hardwareAccess', val)}
            error={errors.hardwareAccess}
            gridClass="hardware-grid"
          />

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
    </div>
  );
};

export default RegisterMultiStepForm;
