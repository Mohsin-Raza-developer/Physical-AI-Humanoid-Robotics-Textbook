import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';
import BaseAuthForm from './BaseAuthForm';

interface LoginFormProps {
  onLogin?: () => void;
}

interface FormData {
  email: string;
  password: string;
}

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const LoginForm: React.FC<LoginFormProps> = ({ onLogin }) => {
  const { siteConfig } = useDocusaurusContext();
  const apiBaseUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:3002';

  const { login } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);
  const [fieldStatus, setFieldStatus] = useState<Record<string, 'valid' | 'invalid' | 'empty'>>({
    email: 'empty',
    password: 'empty',
  });
  const emailRef = useRef<HTMLInputElement>(null);

  // Auto-focus on email field when component mounts
  useEffect(() => {
    if (emailRef.current) {
      emailRef.current.focus();
    }
  }, []);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value,
    });

    // Real-time validation
    if (name === 'email') {
      setFieldStatus(prev => ({
        ...prev,
        email: value && /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(value) ? 'valid' : value ? 'invalid' : 'empty'
      }));
    } else if (name === 'password') {
      setFieldStatus(prev => ({
        ...prev,
        password: value ? 'valid' : 'empty'
      }));
    }

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors({
        ...errors,
        [name]: '',
      });
    }
  };

  const togglePasswordVisibility = () => {
    setShowPassword(!showPassword);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setErrors({});

    try {
      const response = await axios.post(`${apiBaseUrl}/api/auth/login`, formData, {
        withCredentials: true
      });

      if (response.status === 200) {
        if (onLogin) onLogin();

        // Use AuthProvider login method to update state and localStorage properly
        const userData = {
          userId: response.data.data?.userId,
          email: response.data.data?.email,
          firstName: response.data.data?.firstName,
          lastName: response.data.data?.lastName,
          token: response.data.data?.token
        };

        login(userData);

        // Redirect to book intro page after successful login
        if (typeof window !== 'undefined') {
          window.location.href = window.location.origin + '/Physical-AI-Humanoid-Robotics-Textbook/docs/intro/';
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
        } else if (status === 401) {
          setErrors({ general: data.message });
        } else {
          setErrors({ general: data.message || 'An error occurred during login' });
        }
      } else {
        setErrors({ general: 'Network error. Please try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <BaseAuthForm
      title="Sign In to Your Account"
      subtitle="Welcome back! Please enter your details."
      onSubmit={handleSubmit}
      errors={errors}
      loading={loading}
      submitButtonText="Sign in"
    >
      <div className="form-group">
        <div className="input-wrapper">
          <div className="input-icon">📨</div>
          <input
            ref={emailRef}
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            placeholder="Enter your email"
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
            value={formData.password}
            onChange={handleChange}
            required
            placeholder="Enter your password"
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
        {errors.password && (
          <div id="password-error" className="field-error">
            {errors.password}
          </div>
        )}
      </div>

      <div className="form-options">
        <label className="checkbox-label">
          <input type="checkbox" name="remember" />
          <span className="checkmark"></span>
          Remember me
        </label>
        <Link to="/auth/forgot-password" className="form-link">Forgot password?</Link>
      </div>
    </BaseAuthForm>
  );
};

export default LoginForm;