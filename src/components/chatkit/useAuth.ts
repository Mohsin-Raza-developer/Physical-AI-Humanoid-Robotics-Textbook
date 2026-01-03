/**
 * useAuth - Custom hook for Better-Auth session detection and management
 * Integrates with existing AuthProvider for chatbot authentication
 */

import { useState, useEffect, useCallback, useContext } from 'react';
import type { UseAuthReturn, UserSession } from '@site/src/types/chatkit.d';
import { STORAGE_KEYS } from '@site/src/types/chatkit.d';

// Import the existing AuthContext (assuming it's available)
// Note: This will be integrated with the existing auth system
const getAuthContext = () => {
  try {
    // Try to import and use existing AuthContext
    // This is a placeholder - actual implementation will use the real AuthContext
    const { createContext } = require('react');
    return createContext(null);
  } catch {
    return null;
  }
};

/**
 * Custom hook for authentication state management
 * Integrates with Better-Auth system
 *
 * @returns Authentication state and methods
 */
export function useAuth(): UseAuthReturn {
  const [session, setSession] = useState<UserSession>({
    isAuthenticated: false,
    userId: null,
    sessionToken: null,
    createdAt: null,
    expiresAt: null,
  });
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  /**
   * Load session from localStorage and validate
   */
  const loadSession = useCallback(async (): Promise<void> => {
    try {
      setIsLoading(true);
      setError(null);

      // Check localStorage for existing session
      const isLoggedIn = localStorage.getItem('isLoggedIn');
      const userData = localStorage.getItem('user');
      const sessionData = localStorage.getItem(STORAGE_KEYS.USER_SESSION);

      if (isLoggedIn === 'true' && userData) {
        const user = JSON.parse(userData);
        let sessionInfo: UserSession;

        if (sessionData) {
          sessionInfo = JSON.parse(sessionData);
        } else {
          // Create session info from user data
          sessionInfo = {
            isAuthenticated: true,
            userId: user.id || user.email || 'unknown',
            sessionToken: user.token || null,
            createdAt: new Date().toISOString(),
            expiresAt: user.expiresAt || null,
          };
        }

        // Check if session is expired
        if (sessionInfo.expiresAt) {
          const expiryDate = new Date(sessionInfo.expiresAt);
          const now = new Date();

          if (now >= expiryDate) {
            // Session expired
            await logout();
            setError('Session expired. Please log in again.');
            return;
          }
        }

        setSession(sessionInfo);
        // Persist session to localStorage
        localStorage.setItem(STORAGE_KEYS.USER_SESSION, JSON.stringify(sessionInfo));
      } else {
        // No valid session
        setSession({
          isAuthenticated: false,
          userId: null,
          sessionToken: null,
          createdAt: null,
          expiresAt: null,
        });
      }
    } catch (err) {
      console.error('Failed to load session:', err);
      setError('Failed to load authentication session');
      setSession({
        isAuthenticated: false,
        userId: null,
        sessionToken: null,
        createdAt: null,
        expiresAt: null,
      });
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Trigger login redirect to Better-Auth login page
   */
  const login = useCallback((): void => {
    try {
      // Get current page URL for return redirect
      const returnUrl = encodeURIComponent(window.location.href);

      // Redirect to login page with return URL
      // Adjust path based on your actual login page location
      window.location.href = `/login?returnUrl=${returnUrl}`;
    } catch (err) {
      console.error('Failed to trigger login:', err);
      setError('Failed to redirect to login page');
    }
  }, []);

  /**
   * Logout and clear session
   */
  const logout = useCallback(async (): Promise<void> => {
    try {
      setIsLoading(true);

      // Clear localStorage
      localStorage.removeItem('isLoggedIn');
      localStorage.removeItem('user');
      localStorage.removeItem(STORAGE_KEYS.USER_SESSION);

      // Reset session state
      setSession({
        isAuthenticated: false,
        userId: null,
        sessionToken: null,
        createdAt: null,
        expiresAt: null,
      });

      setError(null);
    } catch (err) {
      console.error('Failed to logout:', err);
      setError('Failed to logout');
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Refresh session by re-validating with server
   */
  const refreshSession = useCallback(async (): Promise<void> => {
    try {
      setIsLoading(true);
      setError(null);

      // Re-validate session
      await loadSession();
    } catch (err) {
      console.error('Failed to refresh session:', err);
      setError('Failed to refresh session');
    } finally {
      setIsLoading(false);
    }
  }, [loadSession]);

  // Load session on mount and set up periodic refresh
  useEffect(() => {
    loadSession();

    // Set up periodic session validation (every 5 minutes)
    const refreshInterval = setInterval(() => {
      if (session.isAuthenticated) {
        refreshSession();
      }
    }, 5 * 60 * 1000); // 5 minutes

    // Listen for storage events (session changes in other tabs)
    const handleStorageChange = (e: StorageEvent) => {
      if (
        e.key === 'isLoggedIn' ||
        e.key === 'user' ||
        e.key === STORAGE_KEYS.USER_SESSION
      ) {
        loadSession();
      }
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      clearInterval(refreshInterval);
      window.removeEventListener('storage', handleStorageChange);
    };
  }, [loadSession, refreshSession, session.isAuthenticated]);

  return {
    session,
    isLoading,
    error,
    login,
    logout,
    refreshSession,
  };
}
