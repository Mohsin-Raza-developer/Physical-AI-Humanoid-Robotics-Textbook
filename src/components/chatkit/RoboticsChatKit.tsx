/**
 * RoboticsChatKit Component
 *
 * ChatKit React component integrated with self-hosted backend.
 * Features:
 * - Connects to FastAPI ChatKit backend at /chatkit endpoint
 * - Integrates with Better Auth authentication
 * - Supports WebSocket streaming (SSE)
 * - Thread persistence via localStorage
 * - Error handling and analytics
 */

import React, { useState, useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../auth/AuthProvider';
import styles from './RoboticsChatKit.module.css';

interface RoboticsChatKitProps {
  className?: string;
  theme?: 'light' | 'dark';
  enableHistory?: boolean;
  enablePersistence?: boolean;
  onError?: (error: Error) => void;
  onReady?: () => void;
}

export function RoboticsChatKit({
  className,
  theme = 'dark',
  enableHistory = true,
  enablePersistence = true,
  onError,
  onReady,
}: RoboticsChatKitProps) {
  const { siteConfig } = useDocusaurusContext();
  const { state: authState } = useAuth();
  const [isReady, setIsReady] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [orientation, setOrientation] = useState<'portrait' | 'landscape'>('portrait');
  // HYDRATION FIX: Track client-side mounting
  const [isMounted, setIsMounted] = useState(false);

  // Get user ID for thread persistence
  const userId = authState.isLoggedIn ? authState.user?.id || null : null;

  // Get API base URL from Docusaurus config
  const apiBaseUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:8000';
  const chatkitUrl = `${apiBaseUrl}/chatkit`;

  // Get user-specific localStorage key for thread persistence
  const getThreadStorageKey = (): string | null => {
    if (!userId) return null;
    return `chatkit-thread-${userId}`;
  };

  // Track current thread ID in state
  const [currentThreadId, setCurrentThreadId] = useState<string | null>(null);

  // HYDRATION FIX: Set mounted state only on client-side
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Load saved thread ID when component mounts or user changes
  useEffect(() => {
    if (!isMounted || !userId) {
      setCurrentThreadId(null);
      return;
    }

    const storageKey = getThreadStorageKey();
    if (!storageKey) return;

    const savedThreadId = localStorage.getItem(storageKey);
    console.log('[ChatKit] 📂 Loading saved thread ID:', savedThreadId);
    setCurrentThreadId(savedThreadId);
  }, [isMounted, userId]);

  // Setup ChatKit with useChatKit hook
  // Memoize configuration to prevent ChatKit re-initialization on renders
  const chatKitConfig = React.useMemo(() => ({
    api: {
      url: chatkitUrl,
      domainKey: 'domain_pk_694ffcc3bc648197a8a990ed8ddac44000a1b891b34ccd6d',
    },

    // Theme configuration - optimized for robotics content
    theme: {
      colorScheme: theme,
      radius: 'soft' as const,
      density: 'normal' as const,
      color: {
        accent: {
          primary: '#4F46E5',  // Deeper Indigo for consistent premium feel
          level: 2,
        },
        grayscale: {
          hue: 220,
          tint: 5,
          shade: 0,
        },
      },
      typography: {
        baseSize: 15,
        fontFamily: 'Inter, system-ui, -apple-system, sans-serif',
        fontFamilyMono: 'Fira Code, monospace',
      },
    },

    // Header configuration with custom title
    header: {
      enabled: true,
      title: {
        text: 'Robotics AI Assistant',
      },
    },

    // History panel configuration
    history: {
      enabled: enableHistory,
      showDelete: true,
      showRename: true,
    },

    // Enhanced start screen with robotics-focused prompts
    startScreen: {
      greeting: '👋 Hello! I am your research companion for Physical AI & Humanoid Robotics.',
      prompts: [
        { label: '🤖 Explain Concepts', prompt: 'Explain the concept of Inverse Kinematics in simple terms.' },
        { label: '📚 Search Textbook', prompt: 'Find information about Zero Moment Point (ZMP) in the book.' },
        { label: '💻 Generate Code', prompt: 'Write a Python script for a PID controller.' },
        { label: '❓ Quiz Me', prompt: 'Ask me a quiz question about sensor fusion.' },
      ],
    },

    // Composer (message input) configuration
    composer: {
      placeholder: 'Ask about robotics, AI, or search the textbook...',
    },

    threadItemActions: {
      feedback: true,
      retry: true,
    },

    // Event handlers
    onReady: () => {
      console.log('[ChatKit] ✅ Ready');
      setIsReady(true);
      onReady?.();
    },

    onError: ({ error }: { error: any }) => {
      console.error('[ChatKit] Error:', error);

      // Provide user-friendly error messages
      let userMessage = error.message || 'Unknown error';

      if (userMessage.includes('network') || userMessage.includes('fetch')) {
        userMessage = '🌐 Network error. Please check your internet connection and try again.';
      } else if (userMessage.includes('auth') || userMessage.includes('unauthorized')) {
        userMessage = '🔒 Authentication error. Please log in again.';
      } else if (userMessage.includes('timeout')) {
        userMessage = '⏱️ Request timeout. The server took too long to respond.';
      } else if (userMessage.includes('rate limit')) {
        userMessage = '⚠️ Too many requests. Please wait a moment and try again.';
      } else if (userMessage.includes('server') || userMessage.includes('500')) {
        userMessage = '🔧 Server error. Our team has been notified. Please try again later.';
      }

      setError(userMessage);
      onError?.(error);
    },

    onThreadChange: ({ threadId }: { threadId: string | null }) => {
      console.log('[ChatKit] 🔄 Thread changed:', threadId);

      // Persist thread ID per user
      const storageKey = getThreadStorageKey();
      if (enablePersistence && storageKey) {
        const previousThreadId = localStorage.getItem(storageKey);

        if (threadId) {
          if (previousThreadId && previousThreadId !== threadId) {
            console.warn(`[ChatKit] ⚠️ THREAD ID CHANGED!`);
          }
          localStorage.setItem(storageKey, threadId);
        } else {
          localStorage.removeItem(storageKey);
        }
      }

      // Analytics tracking
      if (typeof window !== 'undefined' && (window as any).gtag) {
        (window as any).gtag('event', 'chatkit_thread_change', {
          thread_id: threadId,
          user_id: userId,
        });
      }
    },

    onResponseStart: () => {
      console.log('[ChatKit] 🤖 AI response started');
    },

    onResponseEnd: () => {
      console.log('[ChatKit] ✅ AI response completed');
    },

    onThreadLoadStart: ({ threadId }: { threadId: string }) => {
      console.log('[ChatKit] 📥 Loading thread:', threadId);
    },

    onThreadLoadEnd: ({ threadId }: { threadId: string }) => {
      console.log('[ChatKit] ✅ Thread loaded:', threadId);
    },
  }), [
    chatkitUrl,
    theme,
    enableHistory,
    enablePersistence,
    userId,
    // getThreadStorageKey is a dependency but should be stable or ref-stable. 
    // It's defined inside RoboticsChatKit but depends on userId which is here. 
    // Function itself is not dependent on other props.
    // However, it is re-created every render. We should not include it in dep array if we don't want re-creation.
    // Instead, we can memoize it or just check userId inside the callback.
    // Since we can't easily change getThreadStorageKey, let's include other stable vars.
    // Note: getThreadStorageKey depends on userId. We already have userId here.
    // We can just inline the logic inside onThreadChange to reduce dependencies.
    // But since this is a huge block, I'll assume getThreadStorageKey is effectively stable if userId is.
    // Actually, I'll refrain from adding it to dependencies since it's defined in the component body
    // and would break memoization. Instead I'll just use it efficiently or rely on userId.
    onError,
    onReady
  ]);

  // Setup ChatKit with useChatKit hook
  const {
    control,
    setThreadId,
    sendUserMessage,
    focusComposer,
  } = useChatKit(chatKitConfig);

  // Initial thread loading logic
  useEffect(() => {
    if (!isReady) return;

    const hasLoadedThread = sessionStorage.getItem(`chatkit-loaded-${userId}`);

    if (enablePersistence && currentThreadId && !hasLoadedThread) {
      console.log(`[ChatKit] � Loading saved thread (FIRST TIME): ${currentThreadId}`);

      // Mark as loaded to prevent re-loading
      sessionStorage.setItem(`chatkit-loaded-${userId}`, currentThreadId);

      // Use setTimeout to allow internal init to complete
      const timeoutId = setTimeout(() => {
        console.log(`[ChatKit] 🚀 Calling setThreadId: ${currentThreadId}`);
        setThreadId(currentThreadId);
      }, 500);

      return () => clearTimeout(timeoutId);
    } else if (hasLoadedThread) {
      console.log(`[ChatKit] ✓ Thread already loaded, skipping setThreadId`);
    } else {
      console.log(`[ChatKit] 🆕 No saved thread, starting fresh conversation`);
    }
  }, [isReady, enablePersistence, currentThreadId, userId, setThreadId]);

  // Auto-focus composer on mount (optional)
  useEffect(() => {
    if (isReady) {
      focusComposer();
    }
  }, [isReady, focusComposer]);

  // Handle auth state changes
  useEffect(() => {
    if (authState.isLoggedIn) {
      console.log('[ChatKit] User authenticated:', authState.user?.email);
    } else {
      console.log('[ChatKit] User not authenticated - using anonymous mode');
    }
  }, [authState.isLoggedIn, authState.user]);

  // T019: Orientation change handling for small screens
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const detectOrientation = () => {
      // Use matchMedia for more reliable orientation detection
      const isPortrait = window.matchMedia('(orientation: portrait)').matches;
      const newOrientation = isPortrait ? 'portrait' : 'landscape';

      if (newOrientation !== orientation) {
        setOrientation(newOrientation);
        console.log('[ChatKit] Orientation changed to:', newOrientation);

        // Analytics tracking
        if ((window as any).gtag) {
          (window as any).gtag('event', 'chatkit_orientation_change', {
            orientation: newOrientation,
            screen_width: window.innerWidth,
            screen_height: window.innerHeight,
          });
        }
      }
    };

    // Initial detection
    detectOrientation();

    // Listen for orientation changes
    window.addEventListener('orientationchange', detectOrientation);
    window.addEventListener('resize', detectOrientation);

    return () => {
      window.removeEventListener('orientationchange', detectOrientation);
      window.removeEventListener('resize', detectOrientation);
    };
  }, [orientation]);

  return (
    <div
      className={`${styles.chatkitContainer} ${className || ''}`}
      data-orientation={orientation}
    >
      {/* Error banner */}
      {error && (
        <div className={styles.errorBanner}>
          <span className={styles.errorIcon}>⚠️</span>
          <span className={styles.errorMessage}>{error}</span>
          <button
            className={styles.errorDismiss}
            onClick={() => setError(null)}
            aria-label="Dismiss error"
          >
            ✕
          </button>
        </div>
      )}

      {/* ChatKit component */}
      <div>
        <ChatKit
          key="chatkit-embedded"
          control={control}
          className={styles.chatkit}
          role="region"
          aria-label="Robotics textbook AI assistant"
        />
      </div>

      {/* Loading overlay (optional) */}
      {!isReady && (
        <div className={styles.loadingOverlay}>
          <div className={styles.loadingSpinner} />
          <p>Loading AI assistant...</p>
        </div>
      )}
    </div>
  );
}

export default RoboticsChatKit;
