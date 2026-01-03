/**
 * useConversationHistory - Custom hook for conversation history management
 * Handles history load/save and persistence on widget close/reopen
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import type {
  UseConversationHistoryReturn,
  ConversationThread,
  Message,
} from '@site/src/types/chatkit.d';
import { ConversationManager } from './ConversationManager';

/**
 * Custom hook for managing conversation history persistence
 * Automatically loads/saves conversation to localStorage
 *
 * @param userId - Current user ID (required for conversation ownership)
 * @param autoSave - Whether to automatically save on conversation changes (default: true)
 * @returns Conversation state and management methods
 */
export function useConversationHistory(
  userId: string | null,
  autoSave: boolean = true
): UseConversationHistoryReturn {
  const managerRef = useRef<ConversationManager>(new ConversationManager());
  const [conversation, setConversation] = useState<ConversationThread | null>(null);
  const [error, setError] = useState<string | null>(null);
  const autoSaveTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  /**
   * Load conversation from localStorage
   */
  const load = useCallback((): void => {
    try {
      setError(null);
      const loaded = managerRef.current.load();

      if (loaded) {
        // Validate that conversation belongs to current user
        if (userId && loaded.userId !== userId) {
          console.warn(
            'Loaded conversation belongs to different user, clearing...'
          );
          managerRef.current.clear();
          setConversation(null);
          return;
        }

        setConversation(loaded);
        console.log(
          `Loaded conversation with ${loaded.messages.length} messages`
        );
      } else {
        setConversation(null);
      }
    } catch (err) {
      console.error('Failed to load conversation:', err);
      setError('Failed to load conversation history');
      setConversation(null);
    }
  }, [userId]);

  /**
   * Save conversation to localStorage
   */
  const save = useCallback(
    (thread: ConversationThread): void => {
      try {
        setError(null);
        const success = managerRef.current.save(thread);

        if (success) {
          setConversation(thread);
        } else {
          setError('Failed to save conversation');
        }
      } catch (err) {
        console.error('Failed to save conversation:', err);
        setError('Failed to save conversation');
      }
    },
    []
  );

  /**
   * Clear conversation from localStorage
   */
  const clear = useCallback((): void => {
    try {
      setError(null);
      const success = managerRef.current.clear();

      if (success) {
        setConversation(null);
        console.log('Conversation cleared');
      } else {
        setError('Failed to clear conversation');
      }
    } catch (err) {
      console.error('Failed to clear conversation:', err);
      setError('Failed to clear conversation');
    }
  }, []);

  /**
   * Check if conversation exists in localStorage
   */
  const exists = useCallback((): boolean => {
    return managerRef.current.exists();
  }, []);

  /**
   * Add a message to the conversation and optionally auto-save
   * @param message - Message to add
   * @returns Updated conversation thread
   */
  const addMessage = useCallback(
    (message: Message): ConversationThread | null => {
      if (!userId) {
        setError('Cannot add message: No user ID provided');
        return null;
      }

      try {
        setError(null);
        const updated = managerRef.current.addMessage(message, userId);
        setConversation(updated);

        // Auto-save with debounce
        if (autoSave) {
          if (autoSaveTimeoutRef.current) {
            clearTimeout(autoSaveTimeoutRef.current);
          }

          autoSaveTimeoutRef.current = setTimeout(() => {
            save(updated);
          }, 500); // Debounce 500ms
        }

        return updated;
      } catch (err) {
        console.error('Failed to add message:', err);
        setError('Failed to add message to conversation');
        return null;
      }
    },
    [userId, autoSave, save]
  );

  /**
   * Get message count
   */
  const getMessageCount = useCallback((): number => {
    return managerRef.current.getMessageCount();
  }, []);

  /**
   * Get recent messages
   */
  const getRecentMessages = useCallback((count: number): Message[] => {
    return managerRef.current.getRecentMessages(count);
  }, []);

  /**
   * Get storage size in bytes
   */
  const getStorageSize = useCallback((): number => {
    return managerRef.current.getStorageSize();
  }, []);

  /**
   * Archive current conversation (sets status to 'archived')
   */
  const archive = useCallback((): void => {
    if (!conversation) {
      return;
    }

    try {
      const archived: ConversationThread = {
        ...conversation,
        status: 'archived',
        lastUpdatedAt: new Date().toISOString(),
      };

      save(archived);
      console.log('Conversation archived');
    } catch (err) {
      console.error('Failed to archive conversation:', err);
      setError('Failed to archive conversation');
    }
  }, [conversation, save]);

  /**
   * Reactivate archived conversation (sets status to 'active')
   */
  const reactivate = useCallback((): void => {
    if (!conversation) {
      return;
    }

    try {
      const active: ConversationThread = {
        ...conversation,
        status: 'active',
        lastUpdatedAt: new Date().toISOString(),
      };

      save(active);
      console.log('Conversation reactivated');
    } catch (err) {
      console.error('Failed to reactivate conversation:', err);
      setError('Failed to reactivate conversation');
    }
  }, [conversation, save]);

  /**
   * Load conversation on mount and when userId changes
   */
  useEffect(() => {
    if (userId) {
      load();
    } else {
      // No user ID - clear conversation
      setConversation(null);
    }
  }, [userId, load]);

  /**
   * Save conversation before unmount (cleanup)
   */
  useEffect(() => {
    return () => {
      if (autoSaveTimeoutRef.current) {
        clearTimeout(autoSaveTimeoutRef.current);
      }

      // Save on unmount if auto-save enabled and conversation exists
      if (autoSave && conversation) {
        managerRef.current.save(conversation);
      }
    };
  }, [autoSave, conversation]);

  /**
   * Listen for storage events (conversation changes in other tabs)
   */
  useEffect(() => {
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === managerRef.current['storageKey']) {
        load();
      }
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, [load]);

  return {
    conversation,
    load,
    save,
    clear,
    exists,
    error,
    // Additional utility methods
    addMessage,
    getMessageCount,
    getRecentMessages,
    getStorageSize,
    archive,
    reactivate,
  } as UseConversationHistoryReturn & {
    addMessage: (message: Message) => ConversationThread | null;
    getMessageCount: () => number;
    getRecentMessages: (count: number) => Message[];
    getStorageSize: () => number;
    archive: () => void;
    reactivate: () => void;
  };
}
