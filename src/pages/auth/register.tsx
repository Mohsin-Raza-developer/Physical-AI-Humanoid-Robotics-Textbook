import React from 'react';
import Layout from '@theme/Layout';
import RegisterMultiStepForm from '@site/src/components/auth/RegisterMultiStepForm';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

function RegisterPage() {
  const { siteConfig } = useDocusaurusContext();
  const logoUrl = useBaseUrl('/img/logo.png');

  return (
    <Layout title={`Register - ${siteConfig.title}`} description="Create a new account">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <div className="auth-logo-container">
                  <img src={logoUrl} alt="Physical AI & Humanoid Robotics Logo" className="auth-logo" />
                </div>
              </div>
              <div className="card__body">
                <RegisterMultiStepForm />
                <div className="margin-top--md">
                  <p className="text--center">
                    Already have an account? <Link to="/auth/login" className="link--primary">Sign in</Link>
                  </p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>

      <style jsx>{`
        .auth-logo-container {
          display: flex;
          justify-content: center;
          margin-bottom: -50px;
        }

        .auth-logo {
          height: 150px;
          width: auto;
        }
      `}</style>
    </Layout>
  );
}

export default RegisterPage;