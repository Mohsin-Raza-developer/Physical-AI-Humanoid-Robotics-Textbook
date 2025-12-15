import React from 'react';
import Layout from '@theme/Layout';
import UserProfile from '@site/src/components/auth/UserProfile';
import ProtectedRoute from '@site/src/components/auth/ProtectedRoute';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function ProfilePage() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout title={`Profile - ${siteConfig.title}`} description="Your account profile">
      <main>
        <ProtectedRoute>
          <UserProfile />
        </ProtectedRoute>
      </main>
    </Layout>
  );
}

export default ProfilePage;