import React, { useEffect, useState } from 'react';
import axios from 'axios';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';
import styles from './UserProfile.module.css';

interface UserProfileProps {
  userId?: string;
}

interface UserData {
  id: string;
  email: string;
  first_name?: string;
  last_name?: string;
  software_level: string;
  hardware_access: string;
  created_at: string;
  last_login_at?: string;
  email_verified: boolean;
  is_active: boolean;
  updated_at: string;
}

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const UserProfile: React.FC<UserProfileProps> = ({ userId }) => {
  const { siteConfig } = useDocusaurusContext();
  const apiBaseUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:3002';

  const { state, logout } = useAuth();
  const [user, setUser] = useState<UserData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [deleteLoading, setDeleteLoading] = useState(false);

  useEffect(() => {
    const fetchUserProfile = async () => {
      try {
        const authUser = state.user;
        const targetUserId = userId || authUser?.userId || authUser?.id || localStorage.getItem('userId');

        if (!targetUserId) {
          setError('User ID not available');
          setLoading(false);
          return;
        }

        const token = localStorage.getItem('authToken');

        const response = await axios.get(`${apiBaseUrl}/api/users/profile?userId=${targetUserId}`, {
          withCredentials: true,
          headers: {
            'Authorization': token ? `Bearer ${token}` : ''
          }
        });

        if (response.status === 200) {
          setUser(response.data.data);
        }
      } catch (err: any) {
        console.error('Error fetching user profile:', err);

        // Check if it's a 401 Unauthorized error (session expired or invalid)
        if (err.response?.status === 401) {
          // Session expired or invalid - logout user
          console.log('Session expired - logging out user');
          logout(); // Clear localStorage and auth state

          // Show alert and redirect to login
          alert('Your session has expired. Please login again.');

          // Redirect to login page
          if (typeof window !== 'undefined') {
            window.location.href = window.location.origin + '/Physical-AI-Humanoid-Robotics-Textbook/auth/login';
          }
          return;
        }

        const detailedError = err.response?.data?.message || err.message || 'Error fetching user profile';
        setError(detailedError);
      } finally {
        setLoading(false);
      }
    };

    fetchUserProfile();
  }, [userId]);

  const handleDeleteAccount = async () => {
    if (!user) return;

    if (!window.confirm('Are you sure you want to permanently delete your account? This action cannot be undone and all your data will be removed.')) {
      return;
    }

    setDeleteLoading(true);
    try {
      const response = await axios.delete(`${apiBaseUrl}/api/auth/account`, {
        data: { userId: user.id }, // Keeping data, but removing query param as new backend uses session
        withCredentials: true
      });

      if (response.status === 200) {
        alert('Account deleted successfully.');
        await logout();
        window.location.href = '/Physical-AI-Humanoid-Robotics-Textbook/';
      }
    } catch (err: any) {
      console.error('Error deleting account:', err);
      alert('Failed to delete account. Please try again.');
    } finally {
      setDeleteLoading(false);
    }
  };

  if (loading) {
    return (
      <div className={styles.loading}>
        <div className="spinner"></div>
        <p>Loading your profile...</p>
      </div>
    );
  }

  if (error || !user) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <div className={styles.content} style={{ textAlign: 'center', padding: '60px 20px' }}>
            <div style={{ fontSize: '48px', marginBottom: '20px' }}>⚠️</div>
            <h2 style={{ color: '#e74c3c', marginBottom: '10px' }}>Unable to Load Profile</h2>
            <p className={styles.errorText} style={{ fontSize: '16px', color: '#666', marginBottom: '20px' }}>
              We couldn't fetch your profile data from the server.
            </p>
            <div style={{ background: '#f8f9fa', padding: '15px', borderRadius: '8px', display: 'inline-block', marginBottom: '25px', textAlign: 'left', border: '1px solid #dee2e6' }}>
              <p style={{ margin: 0, fontSize: '12px', color: '#666', fontFamily: 'monospace' }}>
                <strong>Technical Error:</strong> {error || 'Unknown network error'}
              </p>
            </div>
            <br />
            <Link to="/" className="button button--primary button--lg">Return Home</Link>
          </div>
        </div>
      </div>
    );
  }

  const initials = (user.first_name?.[0] || user.email[0]).toUpperCase();
  const fullName = user.first_name && user.last_name
    ? `${user.first_name} ${user.last_name}`
    : user.email.split('@')[0];

  return (
    <div className={styles.container}>
      <div className={styles.card}>

        {/* Header Section */}
        <div className={styles.header}>
          <div className={styles.avatarContainer}>
            <div className={styles.avatar}>{initials}</div>
          </div>
          <div className={styles.info}>
            <h1 className={styles.name}>{fullName}</h1>
            <div className={styles.role}>Registered Member</div>
          </div>
        </div>

        {/* Content Section */}
        <div className={styles.content}>
          <div className={styles.sectionLabel}>Account Information</div>

          <div className={styles.grid}>
            {/* Email */}
            <div className={styles.dataCard}>
              <div className={styles.iconBox}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"></path><polyline points="22,6 12,13 2,6"></polyline></svg>
              </div>
              <div className={styles.dataContent}>
                <div className={styles.label}>Email Address</div>
                <div className={styles.value}>{user.email}</div>
              </div>
            </div>

            {/* Software Level */}
            <div className={styles.dataCard}>
              <div className={styles.iconBox}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="16 18 22 12 16 6"></polyline><polyline points="8 6 2 12 8 18"></polyline></svg>
              </div>
              <div className={styles.dataContent}>
                <div className={styles.label}>Software Level</div>
                <div className={styles.value}>{user.software_level}</div>
              </div>
            </div>

            {/* Hardware Access */}
            <div className={styles.dataCard}>
              <div className={styles.iconBox}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"></rect><line x1="8" y1="21" x2="16" y2="21"></line><line x1="12" y1="17" x2="12" y2="21"></line></svg>
              </div>
              <div className={styles.dataContent}>
                <div className={styles.label}>Hardware Access</div>
                <div className={styles.value}>{user.hardware_access}</div>
              </div>
            </div>

            {/* Member Since */}
            <div className={styles.dataCard}>
              <div className={styles.iconBox}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><rect x="3" y="4" width="18" height="18" rx="2" ry="2"></rect><line x1="16" y1="2" x2="16" y2="6"></line><line x1="8" y1="2" x2="8" y2="6"></line><line x1="3" y1="10" x2="21" y2="10"></line></svg>
              </div>
              <div className={styles.dataContent}>
                <div className={styles.label}>Member Since</div>
                <div className={styles.value}>{new Date(user.created_at).toLocaleDateString()}</div>
              </div>
            </div>
          </div>

          <div className={styles.dangerZone}>
            <div className={styles.dangerBox}>
              <div>
                <div className={styles.dangerTitle}>Delete Account</div>
                <p className={styles.dangerDesc}>Permanently remove your account and all data.</p>
              </div>
              <button
                onClick={handleDeleteAccount}
                className={styles.deleteButton}
                disabled={deleteLoading}
              >
                {deleteLoading ? 'Processing...' : 'Delete Account'}
              </button>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};

export default UserProfile;