/**
 * TypeScript Type Definitions for ChatKit Integration
 * Minimal types needed for ChatKit React component integration
 */

/**
 * Represents a single message in the conversation
 */
export interface Message {
  /** Unique identifier for the message */
  id: string;
  /** Message content */
  content: string;
  /** Message sender: 'user' or 'bot' */
  sender: 'user' | 'bot';
  /** Delivery status of the message */
  status: 'pending' | 'sent' | 'delivered' | 'failed';
  /** Timestamp when message was created (ISO 8601 format) */
  timestamp: string;
}

/**
 * Represents an authenticated user session
 */
export interface UserSession {
  /** User authentication status */
  isAuthenticated: boolean;
  /** Unique user identifier */
  userId: string | null;
  /** Session token for API requests */
  sessionToken: string | null;
  /** Timestamp when session was created (ISO 8601 format) */
  createdAt: string | null;
  /** Timestamp when session expires (ISO 8601 format) */
  expiresAt: string | null;
}

/**
 * Represents a complete conversation session
 */
export interface ConversationThread {
  /** Unique thread identifier for persistence */
  threadId: string;
  /** User identifier linking to authenticated session */
  userId: string;
  /** Collection of messages in this conversation */
  messages: Message[];
  /** Timestamp when conversation was created (ISO 8601 format) */
  createdAt: string;
  /** Timestamp when conversation was last updated (ISO 8601 format) */
  lastUpdatedAt: string;
  /** Conversation status */
  status: 'active' | 'archived';
}

/**
 * Return type for useAuth hook
 */
export interface UseAuthReturn {
  /** Current user session */
  session: UserSession;
  /** Whether auth status is being checked */
  isLoading: boolean;
  /** Error from auth operations */
  error: string | null;
  /** Function to trigger login redirect */
  login: () => void;
  /** Function to logout */
  logout: () => void;
  /** Function to refresh session */
  refreshSession: () => Promise<void>;
}

/**
 * Return type for useConversationHistory hook
 */
export interface UseConversationHistoryReturn {
  /** Current conversation thread */
  conversation: ConversationThread | null;
  /** Load conversation from localStorage */
  load: () => void;
  /** Save conversation to localStorage */
  save: (thread: ConversationThread) => void;
  /** Clear conversation from localStorage */
  clear: () => void;
  /** Check if conversation exists in localStorage */
  exists: () => boolean;
  /** Error from storage operations */
  error: string | null;
}

/**
 * localStorage key schema for chatkit data
 */
export const STORAGE_KEYS = {
  CONVERSATION_THREAD: 'chatkit:conversation:thread',
  USER_SESSION: 'chatkit:user:session',
} as const;

/**
 * Constants for chatbot configuration
 */
export const CHATBOT_CONSTANTS = {
  MAX_MESSAGE_LENGTH: 2000,
  CHARACTER_COUNT_THRESHOLD: 0.8, // Show count at 80%
  MIN_TOUCH_TARGET_SIZE: 44, // pixels (WCAG 2.1)
  MIN_BODY_FONT_SIZE: 14, // pixels
  MIN_INPUT_FONT_SIZE: 16, // pixels
  SMALL_SCREEN_MAX_WIDTH: 360, // pixels (3-4 inch displays)
  MAX_RETRY_ATTEMPTS: 3,
  RECONNECT_DELAY: 2000, // milliseconds
} as const;
