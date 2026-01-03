/**
 * AuthGate Component
 *
 * Restricts chatbot access to authenticated users only.
 * Shows login button for unauthenticated visitors.
 * Integrates with Better-Auth for session management.
 */

import React, { useEffect, useState } from 'react';
import type { AuthGateProps } from '@site/src/types/chatkit.d';
import { useAuth } from './useAuth';
import styles from './AuthGate.module.css';

/**
 * Authentication gate component that controls access to chat interface
 *
 * @param props - AuthGate component props
 * @returns Rendered component (login button or children)
 */
export function AuthGate({
  loginPageUrl = '/login',
  onAuthStateChange,
  children,
}: AuthGateProps): JSX.Element {
  const { session, isLoading, error, login, refreshSession } = useAuth();
  const [showSessionExpired, setShowSessionExpired] = useState(false);
  const [lastAuthState, setLastAuthState] = useState(session.isAuthenticated);

  /**
   * Notify parent component of auth state changes
   */
  useEffect(() => {
    if (onAuthStateChange && lastAuthState !== session.isAuthenticated) {
      onAuthStateChange(session.isAuthenticated);
      setLastAuthState(session.isAuthenticated);
    }
  }, [session.isAuthenticated, lastAuthState, onAuthStateChange]);

  /**
   * Check for session expiration
   */
  useEffect(() => {
    if (error && error.includes('expired')) {
      setShowSessionExpired(true);
    }
  }, [error]);

  /**
   * Periodic session validation (every 2 minutes)
   */
  useEffect(() => {
    if (!session.isAuthenticated) {
      return;
    }

    const validationInterval = setInterval(() => {
      refreshSession();
    }, 2 * 60 * 1000); // 2 minutes

    return () => clearInterval(validationInterval);
  }, [session.isAuthenticated, refreshSession]);

  /**
   * Handle login button click
   */
  const handleLoginClick = () => {
    login();
  };

  /**
   * Handle refresh session after expiration
   */
  const handleRefreshSession = () => {
    setShowSessionExpired(false);
    login();
  };

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.authGate}>
        <div className={styles.loadingContainer}>
          <div className={styles.spinner}></div>
          <p className={styles.loadingText}>Checking authentication...</p>
        </div>
      </div>
    );
  }

  // Session expired state
  if (showSessionExpired && !session.isAuthenticated) {
    return (
      <div className={styles.authGate}>
        <div className={styles.expiredContainer}>
          <div className={styles.expiredIcon}>⏱️</div>
          <h3 className={styles.expiredTitle}>Session Expired</h3>
          <p className={styles.expiredMessage}>
            Your session has expired. Please log in again to continue.
          </p>
          <button
            className={styles.loginButton}
            onClick={handleRefreshSession}
            aria-label="Log in to continue chatting"
          >
            Log In Again
          </button>
        </div>
      </div>
    );
  }

  // Unauthenticated state - show login prompt
  if (!session.isAuthenticated) {
    return (
      <div className={styles.authGate}>
        <div className={styles.loginContainer}>
          <div className={styles.lockIcon}>🔒</div>
          <h3 className={styles.loginTitle}>Login Required</h3>
          <p className={styles.loginMessage}>
            Please log in to access the Physical AI Robotics Assistant.
          </p>
          <button
            className={styles.loginButton}
            onClick={handleLoginClick}
            aria-label="Log in to start chatting"
          >
            Log In
          </button>
          {error && !error.includes('expired') && (
            <p className={styles.errorMessage} role="alert">
              {error}
            </p>
          )}
        </div>
      </div>
    );
  }

  // Authenticated - render children (chat interface)
  return <>{children}</>;
}

export default AuthGate;
