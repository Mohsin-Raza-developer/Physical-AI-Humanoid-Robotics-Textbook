/**
 * ChatKit Floating Widget Component
 *
 * Advanced floating chat button with:
 * - Text selection support (Select Text and ASK)
 * - Page context awareness
 * - Login requirement
 * - Personalization menu
 * - Script loading detection
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { useAuth } from '../auth/AuthProvider';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './ChatKitWidget.module.css';

const isBrowser = typeof window !== 'undefined';

interface ChatKitWidgetProps {
  position?: 'bottom-right' | 'bottom-left';
  buttonText?: string;
}

export function ChatKitWidget({ position = 'bottom-right' }: ChatKitWidgetProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [selectionPosition, setSelectionPosition] = useState<{ x: number; y: number } | null>(null);
  const [showPersonalizeMenu, setShowPersonalizeMenu] = useState(false);
  // HYDRATION FIX: Always start with 'pending' to match server render
  const [scriptStatus, setScriptStatus] = useState<'pending' | 'ready' | 'error'>('pending');
  // HYDRATION FIX: Track client-side mounting
  const [isMounted, setIsMounted] = useState(false);
  const selectionRef = useRef<HTMLDivElement>(null);
  const personalizeMenuRef = useRef<HTMLDivElement>(null);
  const isMountedRef = useRef(true);
  const { state: authState } = useAuth();
  const { siteConfig } = useDocusaurusContext();

  // Debug authentication state
  useEffect(() => {
    console.log('[ChatKit Widget] 🔐 Auth State Check');
  }, [authState]);

  // Get backend URL from siteConfig or default
  // Use apiBaseUrl from docusaurus.config.ts (Railway production or localhost for dev)
  const apiBaseUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'https://app-production-f33e.up.railway.app';

  // Check if user is logged in
  const isLoggedIn = authState.isLoggedIn;
  const user = authState.user;

  // Handle login redirect
  const handleLogin = useCallback(() => {
    // Redirect to login page with base path
    const basePath = siteConfig.baseUrl || '/';
    window.location.href = `${basePath}auth/login`;
  }, [siteConfig.baseUrl]);

  // HYDRATION FIX: Set mounted state only on client-side
  useEffect(() => {
    console.log('[ChatKit Widget] 🚀 Component mounted');
    console.log('[ChatKit Widget] Initial state - isLoggedIn:', isLoggedIn, 'isOpen:', isOpen);
    console.log('[ChatKit Widget] scriptStatus:', scriptStatus);
    setIsMounted(true);
  }, []);

  // Check if ChatKit script is loaded
  useEffect(() => {
    if (!isBrowser || !isMounted) {
      console.log('[ChatKit Widget] ⏸️ Script check skipped - browser:', isBrowser, 'mounted:', isMounted);
      return;
    }

    console.log('[ChatKit Widget] 🔍 Checking for ChatKit script...');
    let timeoutId: number | undefined;

    const handleLoaded = () => {
      if (!isMountedRef.current) return;
      console.log('[ChatKit Widget] ✅ ChatKit script loaded!');
      setScriptStatus('ready');
    };

    const handleError = (event: Event) => {
      console.error('[ChatKit Widget] ❌ Failed to load chatkit.js', event);
      if (!isMountedRef.current) return;
      setScriptStatus('error');
      const detail = (event as CustomEvent<unknown>)?.detail ?? 'unknown error';
      console.error('[ChatKit Widget] Script error detail:', detail);
    };

    window.addEventListener('chatkit-script-loaded', handleLoaded);
    window.addEventListener('chatkit-script-error', handleError as EventListener);

    const hasCustomElement = window.customElements?.get('openai-chatkit');
    console.log('[ChatKit Widget] Custom element check:', hasCustomElement ? 'FOUND' : 'NOT FOUND');

    if (hasCustomElement) {
      handleLoaded();
    } else if (scriptStatus === 'pending') {
      console.log('[ChatKit Widget] ⏳ Waiting for script (15s timeout)...');
      timeoutId = window.setTimeout(() => {
        if (!window.customElements?.get('openai-chatkit')) {
          console.error('[ChatKit Widget] ⏰ Script load timeout!');
          handleError(
            new CustomEvent('chatkit-script-error', {
              detail: 'ChatKit web component is unavailable. Verify that the script URL is reachable.',
            })
          );
        }
      }, 5000);
    }

    return () => {
      window.removeEventListener('chatkit-script-loaded', handleLoaded);
      window.removeEventListener('chatkit-script-error', handleError as EventListener);
      if (timeoutId) {
        window.clearTimeout(timeoutId);
      }
    };
  }, [isMounted, scriptStatus]); // Add isMounted dependency

  useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Get current page context - enhanced for better awareness
  const getPageContext = useCallback(() => {
    if (typeof window === 'undefined') return null;

    // Extract page metadata
    const metaDescription = document.querySelector('meta[name="description"]')?.getAttribute('content') || '';
    const metaKeywords = document.querySelector('meta[name="keywords"]')?.getAttribute('content') || '';

    // Get main content (try to find article or main content area)
    const mainContent = document.querySelector('article') ||
      document.querySelector('main') ||
      document.querySelector('[role="main"]') ||
      document.body;

    // Extract headings for context
    const headings = Array.from(mainContent.querySelectorAll('h1, h2, h3'))
      .slice(0, 5)
      .map(h => h.textContent?.trim())
      .filter(Boolean)
      .join(', ');

    return {
      url: window.location.href,
      title: document.title,
      path: window.location.pathname,
      description: metaDescription,
      keywords: metaKeywords,
      headings: headings,
      timestamp: new Date().toISOString(),
    };
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (!selection || selection.rangeCount === 0) {
        setSelectedText('');
        setSelectionPosition(null);
        return;
      }

      const selectedText = selection.toString().trim();
      if (selectedText.length > 0) {
        setSelectedText(selectedText);

        // Get selection position for "Ask" button placement
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 10, // Position above selection
        });
      } else {
        setSelectedText('');
        setSelectionPosition(null);
      }
    };

    document.addEventListener('selectionchange', handleSelection);
    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('selectionchange', handleSelection);
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Get user identifier (id, email, or name as fallback)
  const getUserIdentifier = useCallback((): string | null => {
    if (!isLoggedIn) return null;

    // Try id first (preferred)
    if (user?.id) return user.id;

    // Fallback to email
    if (user?.email) {
      console.warn('[ChatKit Widget] ⚠️ Using email as user identifier');
      return user.email;
    }

    // Fallback to name
    if (user?.name) {
      console.warn('[ChatKit Widget] ⚠️ Using name as user identifier');
      return user.name;
    }

    // Last resort: check localStorage for authToken and derive ID
    const authToken = localStorage.getItem('authToken');
    if (authToken) {
      console.warn('[ChatKit Widget] ⚠️ User logged in but no identifier found');
      console.warn('[ChatKit Widget] Auth token exists but data incomplete');
    }

    return null;
  }, [isLoggedIn, user?.id, user?.email, user?.name]);

  // Get user-specific localStorage key
  const getThreadStorageKey = useCallback(() => {
    const userId = getUserIdentifier();
    if (!userId) {
      console.warn('[ChatKit Widget] ⚠️ Cannot create storage key - no user identifier');
      return null;
    }
    return `chatkit-thread-${userId}`;
  }, [getUserIdentifier]);

  // Track current thread ID in state to force ChatKit re-initialization
  const [currentThreadId, setCurrentThreadId] = useState<string | null>(null);
  const [isChatkitReady, setIsChatkitReady] = useState(false);

  // Hide scroll indicators (inspired by robolearn approach)
  useEffect(() => {
    if (!isOpen || !isChatkitReady) return;

    const hideScrollIndicators = () => {
      // Find and hide "Scroll X%" text indicators
      const walker = document.createTreeWalker(
        document.body,
        NodeFilter.SHOW_TEXT,
        null
      );

      let node;
      while ((node = walker.nextNode())) {
        const text = node.textContent || '';
        // Match "Scroll X%" pattern
        if (/Scroll\s+\d+%/i.test(text)) {
          const parent = node.parentElement;
          if (parent) {
            parent.style.display = 'none';
            parent.style.visibility = 'hidden';
            parent.style.opacity = '0';
            console.log('[ChatKit Widget] Hidden scroll indicator:', text);
          }
        }
      }
    };

    // Run multiple times with delays (ChatKit may re-render)
    const timers = [50, 200, 500, 1000, 2000].map(delay =>
      setTimeout(hideScrollIndicators, delay)
    );

    // MutationObserver for dynamic elements
    const observer = new MutationObserver(hideScrollIndicators);
    observer.observe(document.body, {
      childList: true,
      subtree: true,
      attributes: true,
      attributeFilter: ['class', 'style']
    });

    return () => {
      timers.forEach(clearTimeout);
      observer.disconnect();
    };
  }, [isOpen, isChatkitReady]);

  // Load saved thread ID when component mounts or auth changes
  useEffect(() => {
    const userId = getUserIdentifier();
    if (!isLoggedIn || !userId) {
      setCurrentThreadId(null);
      return;
    }

    const storageKey = `chatkit-thread-${userId}`;
    const savedThreadId = localStorage.getItem(storageKey);

    console.log('[ChatKit Widget] 📂 Loading saved thread ID');
    setCurrentThreadId(savedThreadId);
  }, [isLoggedIn, getUserIdentifier]);

  // Use refs for unstable dependencies to keep chatKitConfig stable
  const userRef = useRef(user);
  const isLoggedInRef = useRef(isLoggedIn);
  const authStateRef = useRef(authState);

  // Update refs when dependencies change
  useEffect(() => {
    userRef.current = user;
    isLoggedInRef.current = isLoggedIn;
    authStateRef.current = authState;
  }, [user, isLoggedIn, authState]);

  // ChatKit configuration
  // Memoize configuration with minimal dependencies to prevent re-renders
  const chatKitConfig = React.useMemo(() => ({
    api: {
      url: `${apiBaseUrl}/chatkit`,
      domainKey: 'domain_pk_694ffcc3bc648197a8a990ed8ddac44000a1b891b34ccd6d',
      // Custom fetch uses refs to access latest state without re-creating the function
      fetch: async (url: string, options: RequestInit) => {
        const currentUser = userRef.current;
        const currentIsLoggedIn = isLoggedInRef.current;

        // Require login
        if (!currentIsLoggedIn) {
          console.error('[ChatKit Widget] ❌ User not logged in');
          throw new Error('User must be logged in');
        }

        // Get ID safely
        const userId = currentUser?.id || currentUser?.email || currentUser?.name;

        if (!userId) {
          console.error('[ChatKit Widget] ❌ No user identifier');
          throw new Error('User identifier not available');
        }

        const pageContext = getPageContext();

        const userInfo = {
          id: userId,
          name: currentUser?.name || currentUser?.email || 'User',
          email: currentUser?.email,
        };

        let modifiedOptions = { ...options };
        if (modifiedOptions.body && typeof modifiedOptions.body === 'string') {
          try {
            const parsed = JSON.parse(modifiedOptions.body);
            // Inject metadata
            if (parsed.type === 'threads.create' && parsed.params?.input) {
              parsed.params.input.metadata = {
                userId: userId,
                userInfo: userInfo,
                pageContext: pageContext,
                ...parsed.params.input.metadata,
              };
              modifiedOptions.body = JSON.stringify(parsed);
            } else if (parsed.type === 'threads.run' && parsed.params?.input) {
              if (!parsed.params.input.metadata) parsed.params.input.metadata = {};
              parsed.params.input.metadata.userInfo = userInfo;
              parsed.params.input.metadata.pageContext = pageContext;
              modifiedOptions.body = JSON.stringify(parsed);
            }
          } catch (e) { /* ignore */ }
        }

        const authToken = isBrowser ? localStorage.getItem('authToken') : null;

        return fetch(url, {
          ...modifiedOptions,
          headers: {
            ...modifiedOptions.headers,
            'Authorization': authToken ? `Bearer ${authToken}` : '',
            'X-User-ID': userId,
            'X-Page-URL': pageContext?.url || '',
            'Content-Type': 'application/json',
          },
        });
      },
    },
    theme: {
      colorScheme: 'light',
      radius: 'soft' as const,
      density: 'normal' as const,
      color: {
        accent: { primary: '#4F46E5', level: 2 }, // Deeper Indigo for premium feel
        grayscale: { hue: 220, tint: 5, shade: 0 },
      },
      typography: {
        baseSize: 15,
        fontFamily: 'Inter, system-ui, -apple-system, sans-serif',
        fontFamilyMono: 'Fira Code, monospace',
      },
    },
    header: { enabled: true, title: { text: 'Robotics AI Assistant' } },
    startScreen: {
      greeting: '👋 Hello! I am your research companion for Physical AI & Humanoid Robotics.',
      prompts: [
        { label: '🤖 Explain Concepts', prompt: 'Explain the concept of Inverse Kinematics in simple terms.' },
        { label: '📚 Search Textbook', prompt: 'Find information about Zero Moment Point (ZMP) in the book.' },
        { label: '💻 Generate Code', prompt: 'Write a Python script for a PID controller.' },
        { label: '❓ Quiz Me', prompt: 'Ask me a quiz question about sensor fusion.' },
      ],
    },
    composer: { placeholder: 'Ask a question or search related topics...' },

    // Thread persistence
    onThreadChange: ({ threadId }: { threadId: string | null }) => {
      // Use ref to get current user ID without dependency
      const currentUser = userRef.current;
      const userId = currentUser?.id || currentUser?.email || currentUser?.name;

      if (!userId) return;

      const storageKey = `chatkit-thread-${userId}`;
      const previousThreadId = localStorage.getItem(storageKey);

      if (threadId) {
        localStorage.setItem(storageKey, threadId);
      } else {
        localStorage.removeItem(storageKey);
      }
    },
    onError: ({ error }: { error: any }) => console.error('ChatKit error'),
    onReady: () => setIsChatkitReady(true),
    onThreadLoadStart: ({ threadId }: { threadId: string }) => {
      console.log('[ChatKit Widget] 📥 Loading thread');
    },
    onThreadLoadEnd: ({ threadId }: { threadId: string }) => {
      console.log('[ChatKit Widget] ✅ Thread loaded');
    },
    onResponseStart: () => {
      console.log('[ChatKit Widget] 🤖 AI response started');
    },
    onResponseEnd: () => {
      console.log('[ChatKit Widget] ✅ AI response completed');
    },
  }), [apiBaseUrl, getPageContext]); // Minimal dependencies!

  const {
    control,
    setThreadId,
    sendUserMessage,
  } = useChatKit(chatKitConfig);

  // Initial thread loading logic moved to useEffect
  useEffect(() => {
    // Only run if we are ready and have a user
    if (!isChatkitReady || !isLoggedIn) return;

    const userId = getUserIdentifier();
    if (!userId) return;

    const hasLoadedThread = sessionStorage.getItem(`chatkit-loaded-${userId}`);

    if (currentThreadId && !hasLoadedThread) {
      console.log('[ChatKit Widget] 🔄 Loading saved thread (FIRST TIME)');
      sessionStorage.setItem(`chatkit-loaded-${userId}`, currentThreadId);

      setTimeout(() => {
        console.log('[ChatKit Widget] 🚀 Calling setThreadId');
        setThreadId(currentThreadId);
      }, 500);
    } else if (hasLoadedThread) {
      console.log('[ChatKit Widget] ✓ Thread already loaded');
    } else {
      console.log('[ChatKit Widget] 🆕 No saved thread');
    }
  }, [isChatkitReady, currentThreadId, isLoggedIn, setThreadId, getUserIdentifier]);

  // Handle "Ask" button click for selected text
  const handleAskSelectedText = useCallback(async () => {
    if (!selectedText || !isLoggedIn) return;

    // Open chat first if not already open
    if (!isOpen) {
      setIsOpen(true);
      // Wait for ChatKit to initialize
      await new Promise(resolve => setTimeout(resolve, 300));
    }

    // Build message with selected text
    const pageContext = getPageContext();
    let messageText = '';

    if (pageContext) {
      messageText = `Can you explain this from "${pageContext.title}":\n\n"${selectedText}"`;
    } else {
      messageText = `Can you explain this:\n\n"${selectedText}"`;
    }

    // Send message using sendUserMessage hook
    try {
      if (sendUserMessage) {
        await sendUserMessage({
          text: messageText,
          newThread: false,
        });

        // Clear selection after successful send
        setTimeout(() => {
          window.getSelection()?.removeAllRanges();
          setSelectedText('');
          setSelectionPosition(null);
        }, 200);
      }
    } catch (error) {
      console.error('Failed to send message:', error);
    }
  }, [selectedText, isOpen, isLoggedIn, sendUserMessage, getPageContext]);

  // Close ChatKit when clicking outside
  useEffect(() => {
    if (!isOpen) return;

    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;

      // Don't close if clicking inside ChatKit or the button
      if (
        target.closest(`.${styles.chatKitContainer}`) ||
        target.closest(`.${styles.chatButton}`)
      ) {
        return;
      }

      setIsOpen(false);
    };

    // Small delay to avoid immediate close on open
    const timeoutId = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 100);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Close on outside click for text selection
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (selectionRef.current && !selectionRef.current.contains(event.target as Node)) {
        const target = event.target as HTMLElement;
        if (target.closest(`.${styles.chatKitContainer}`)) {
          return;
        }
        setSelectedText('');
        setSelectionPosition(null);
      }
    };

    if (selectedText) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [selectedText]);

  // Handle chat button click
  const handleChatButtonClick = useCallback(() => {
    console.log('[ChatKit Widget] 🖱️ Button clicked!');
    console.log('[ChatKit Widget] isLoggedIn:', isLoggedIn);
    console.log('[ChatKit Widget] isOpen:', isOpen);

    if (!isLoggedIn) {
      console.log('[ChatKit Widget] Redirecting to login...');
      handleLogin();
      return;
    }

    console.log('[ChatKit Widget] Toggling chat window...');
    setIsOpen(!isOpen);
    setShowPersonalizeMenu(false);
  }, [isLoggedIn, isOpen, handleLogin]);

  // Handle personalize menu actions
  const handlePersonalize = useCallback(() => {
    setShowPersonalizeMenu(false);
    setIsOpen(true);
    setTimeout(async () => {
      if (sendUserMessage) {
        await sendUserMessage({
          text: "How can I personalize my learning experience?",
          newThread: false,
        });
      }
    }, 500);
  }, [sendUserMessage]);

  // Close personalize menu when clicking outside
  useEffect(() => {
    if (!showPersonalizeMenu) return;

    const handleClickOutside = (event: MouseEvent) => {
      if (
        personalizeMenuRef.current &&
        !personalizeMenuRef.current.contains(event.target as Node) &&
        !(event.target as HTMLElement).closest(`.${styles.chatButton}`)
      ) {
        setShowPersonalizeMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showPersonalizeMenu]);

  return (
    <>
      {/* Floating Chat Button and Settings */}
      <div className={`${styles.chatButtonContainer} ${position === 'bottom-right' ? styles.bottomRight : styles.bottomLeft}`}>
        {/* Settings/Personalize Button - Visible when logged in */}
        {isLoggedIn && (
          <button
            className={styles.settingsButton}
            onClick={() => setShowPersonalizeMenu(!showPersonalizeMenu)}
            aria-label="Personalize Assistant"
            title="Personalize Assistant"
          >
            <svg
              width="18"
              height="18"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <circle cx="12" cy="12" r="3" />
              <path d="M12 1v6m0 6v6M5.64 5.64l4.24 4.24m4.24 4.24l4.24 4.24M1 12h6m6 0h6M5.64 18.36l4.24-4.24m4.24-4.24l4.24-4.24" />
            </svg>
          </button>
        )}

        <button
          className={styles.chatButton}
          onClick={handleChatButtonClick}
          aria-label={isLoggedIn ? "Open AI Assistant" : "Login to use AI Assistant"}
          title={isLoggedIn ? "Open AI Assistant" : "Login to use AI Assistant"}
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>

        {/* Personalize Menu */}
        {showPersonalizeMenu && isLoggedIn && (
          <div
            ref={personalizeMenuRef}
            className={styles.personalizeMenu}
          >
            <div className={styles.personalizeMenuHeader}>
              <h4>Personalize Assistant</h4>
              <button
                className={styles.closeMenuButton}
                onClick={() => setShowPersonalizeMenu(false)}
                aria-label="Close menu"
              >
                ×
              </button>
            </div>
            <div className={styles.personalizeMenuContent}>
              <p>Customize your AI assistant:</p>
              <button
                className={styles.personalizeButton}
                onClick={handlePersonalize}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M12 2L2 7l10 5 10-5-10-5z" />
                  <path d="M2 17l10 5 10-5" />
                  <path d="M2 12l10 5 10-5" />
                </svg>
                Set Learning Preferences
              </button>
              <button
                className={styles.personalizeButton}
                onClick={() => {
                  setShowPersonalizeMenu(false);
                  setIsOpen(true);
                  setTimeout(async () => {
                    if (sendUserMessage) {
                      await sendUserMessage({
                        text: "What can you help me with?",
                        newThread: false,
                      });
                    }
                  }, 500);
                }}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10" />
                  <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
                  <path d="M12 17h.01" />
                </svg>
                What Can You Help With?
              </button>
            </div>
          </div>
        )}
      </div>

      {/* ChatKit Component - Floating widget */}
      {/* Only render when script is ready and user is logged in */}
      {isOpen && scriptStatus === 'ready' && isLoggedIn && (
        <div className={styles.chatKitContainer}>
          <ChatKit
            key="chatkit-widget"
            control={control}
            className={styles.chatKit}
          />
        </div>
      )}

      {/* Login Prompt Overlay */}
      {isOpen && !isLoggedIn && (
        <div className={styles.loginPrompt}>
          <div className={styles.loginPromptContent}>
            <h3>Login Required</h3>
            <p>Please log in to use the AI Assistant and get personalized help with your robotics learning journey.</p>
            <button
              className={styles.loginButton}
              onClick={handleLogin}
            >
              Log In
            </button>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              Close
            </button>
          </div>
        </div>
      )}

      {/* "Ask" Button for Selected Text */}
      {selectedText && selectionPosition && isLoggedIn && (
        <div
          ref={selectionRef}
          className={styles.askButton}
          style={{
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y}px`,
            transform: 'translateX(-50%)',
          }}
          onClick={(e) => {
            e.stopPropagation();
            e.preventDefault();
            handleAskSelectedText();
          }}
          onMouseDown={(e) => {
            e.stopPropagation();
            e.preventDefault();
          }}
        >
          <svg
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
          <span>Ask</span>
        </div>
      )}
    </>
  );
}

export default ChatKitWidget;
