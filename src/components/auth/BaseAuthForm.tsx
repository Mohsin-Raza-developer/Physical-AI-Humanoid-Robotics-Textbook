import React from 'react';

interface BaseAuthFormProps {
  title: string;
  subtitle: string;
  children: React.ReactNode;
  onSubmit: (e: React.FormEvent) => void;
  errors?: Record<string, string>;
  successMessage?: string;
  loading?: boolean;
  submitButtonText: string;
  formClass?: string;
}

const BaseAuthForm: React.FC<BaseAuthFormProps> = ({
  title,
  subtitle,
  children,
  onSubmit,
  errors,
  successMessage,
  loading = false,
  submitButtonText,
  formClass = 'auth-form'
}) => {
  return (
    <div className="auth-form-container">
      <form onSubmit={onSubmit} className={formClass}>
        <div className="form-header">
          <h2>{title}</h2>
          <p className="form-subtitle">{subtitle}</p>
        </div>

        {errors?.general && (
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

        {children}

        <button type="submit" disabled={loading} className="auth-button">
          {loading ? (
            <span className="loading-spinner">
              <span className="spinner"></span> {submitButtonText}...
            </span>
          ) : (
            submitButtonText
          )}
        </button>
      </form>

    </div>
  );
};

export default BaseAuthForm;