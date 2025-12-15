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

      <style jsx>{`
        /* Premium Glassmorphism Design */
        /* Premium Glassmorphism Design */
        .auth-form-container {
          width: 100%;
          display: flex;
          flex-direction: column;
          justify-content: center;
          align-items: center;
          padding: 10px;
        }

        .auth-form {
          width: 100%;
          max-width: 450px;
          background: var(--ifm-card-background-color);
          backdrop-filter: blur(20px);
          -webkit-backdrop-filter: blur(20px);
          border-radius: 20px;
          padding: 40px;
          box-shadow: 0 20px 80px rgba(32, 199, 221, 0.14);
          border: 1px solid var(--ifm-color-emphasis-200);
          position: relative;
          overflow: hidden;
        }

        .form-header {
          text-align: center;
          margin-bottom: 30px;
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

        .password-toggle, .toggle-password-btn {
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

        .password-toggle:hover, .toggle-password-btn:hover {
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

        input.valid {
          border-color: var(--ifm-color-success);
        }

        input.invalid, input.error {
          border-color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-lightest);
        }

        .error-toast,
        .success-toast {
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

        .error-toast-icon,
        .success-toast-icon {
          margin-right: 10px;
          font-size: 1.2rem;
        }

        .error-toast-message,
        .success-toast-message {
          font-weight: 500;
        }

        .error-toast-message {
          color: var(--ifm-color-danger-dark);
        }

        .success-toast-message {
          color: var(--ifm-color-success-dark);
        }

        .field-error {
          color: var(--ifm-color-danger);
          font-size: 0.85rem;
          margin-top: 8px;
          display: flex;
          align-items: center;
          gap: 6px;
        }

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
          position: relative;
          box-shadow: 0 10px 20px rgba(var(--ifm-color-primary-rgb), 0.2);
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

        .form-options {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin: 20px 0;
          font-size: 0.9rem;
        }

        .checkbox-label {
          display: flex;
          align-items: center;
          cursor: pointer;
          color: var(--ifm-color-emphasis-700);
          font-weight: 500;
        }

        .checkbox-label input[type="checkbox"] {
          display: none;
        }

        .checkmark {
          width: 20px;
          height: 20px;
          border: 2px solid var(--ifm-color-emphasis-400);
          border-radius: 6px;
          margin-right: 10px;
          position: relative;
          transition: all 0.2s ease;
        }

        .checkbox-label input[type="checkbox"]:checked + .checkmark {
          background-color: var(--ifm-color-primary);
          border-color: var(--ifm-color-primary);
        }

        .checkbox-label input[type="checkbox"]:checked + .checkmark::after {
          content: "";
          position: absolute;
          left: 6px;
          top: 2px;
          width: 6px;
          height: 12px;
          border: solid white;
          border-width: 0 2px 2px 0;
          transform: rotate(45deg);
        }

        .checkmark:hover {
          border-color: var(--ifm-color-primary);
        }

        .form-link {
          color: var(--ifm-color-primary);
          text-decoration: none;
          transition: color 0.2s ease;
          font-weight: 600;
        }

        .form-link:hover {
          color: var(--ifm-color-primary-darker);
          text-decoration: underline;
        }

        @media (max-width: 480px) {
          .auth-form {
            padding: 24px;
            border-radius: 16px;
          }

          .auth-button {
            padding: 16px;
          }

          .form-options {
            flex-direction: column;
            align-items: flex-start;
            gap: 10px;
          }
        }
      `}</style>
    </div>
  );
};

export default BaseAuthForm;