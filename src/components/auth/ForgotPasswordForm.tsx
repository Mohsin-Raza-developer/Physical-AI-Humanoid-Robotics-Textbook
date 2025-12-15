import React, { useState } from 'react';
import axios from 'axios';
import Link from '@docusaurus/Link';
import BaseAuthForm from './BaseAuthForm';

interface ForgotPasswordFormProps {
  onSuccess?: () => void;
}

const NEXT_PUBLIC_API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).env?.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'
  : process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3002'; // Backend runs on port 3002

const ForgotPasswordForm: React.FC<ForgotPasswordFormProps> = ({ onSuccess }) => {
  const [email, setEmail] = useState('');
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setErrors({});
    setSuccessMessage('');

    try {
      const response = await axios.post(`${NEXT_PUBLIC_API_BASE_URL}/api/auth/reset-password/request`, { email });

      if (response.status === 200) {
        setSuccessMessage('Password reset email sent if email exists. Please check your inbox.');
        if (onSuccess) onSuccess();
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
          setErrors({ general: data.message || 'An error occurred requesting password reset' });
        }
      } else {
        setErrors({ general: 'Network error. Please try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    if (name === 'email') setEmail(value);
    
    // Clear error when user starts typing
    if (errors[name]) {
      setErrors({
        ...errors,
        [name]: '',
      });
    }
  };

  return (
    <BaseAuthForm
      title="Reset Your Password"
      subtitle="Enter your email and we'll send you a link to reset your password."
      onSubmit={handleSubmit}
      errors={errors}
      successMessage={successMessage}
      loading={loading}
      submitButtonText="Send Reset Link"
    >
      <div className="form-group">
        <div className="input-wrapper">
          <div className="input-icon">✉️</div>
          <input
            type="email"
            id="email"
            name="email"
            value={email}
            onChange={handleChange}
            required
            placeholder="Enter your email address"
            className={errors.email ? 'error' : ''}
            aria-describedby={errors.email ? "email-error" : undefined}
          />
        </div>
        {errors.email && (
          <div id="email-error" className="field-error">
            {errors.email}
          </div>
        )}
      </div>
    </BaseAuthForm>
  );
};

export default ForgotPasswordForm;