import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '@site/src/components/auth/LoginForm';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

function LoginPage() {
  const { siteConfig } = useDocusaurusContext();
  const logoUrl = useBaseUrl('/img/logo.png');

  return (
    <Layout title={`Login - ${siteConfig.title}`} description="Login to your account">
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
                <LoginForm />
                <div className="margin-top--md">
                  <p className="text--center">
                    Don't have an account? <Link to="/auth/register" className="link--primary">Sign up</Link>
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

export default LoginPage;