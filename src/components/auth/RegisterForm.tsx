import React from 'react';
import Link from '@docusaurus/Link';

// This component now displays a message directing users to the improved multi-step form
const RegisterForm = () => {
  return (
    <div className="register-form-upgrade-notice">
      <div className="upgrade-message">
        <h3>Enhanced Registration Experience Available</h3>
        <p>For a better registration experience with guided steps and improved UX, please use our new multi-step form.</p>
        <div className="button-container">
          <Link to="/auth/register" className="button button--primary">
            Go to Multi-Step Registration
          </Link>
        </div>
      </div>

      <style jsx>{`
        .register-form-upgrade-notice {
          text-align: center;
          padding: 20px;
        }
        
        .upgrade-message {
          max-width: 500px;
          margin: 0 auto;
          padding: 30px;
          background: var(--ifm-color-emphasis-100);
          border-radius: 8px;
          box-shadow: var(--ifm-global-shadow-lw);
        }
        
        .upgrade-message h3 {
          color: var(--ifm-heading-color);
          margin-bottom: 15px;
        }
        
        .upgrade-message p {
          margin-bottom: 20px;
          color: var(--ifm-color-emphasis-700);
        }
        
        .button-container {
          text-align: center;
        }
      `}</style>
    </div>
  );
};

export default RegisterForm;