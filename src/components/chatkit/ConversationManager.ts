/**
 * ConversationManager - Manages conversation history with localStorage persistence
 * Handles indefinite retention of conversation threads until manually cleared
 */

import type { ConversationThread, Message } from '@site/src/types/chatkit.d';
import { STORAGE_KEYS } from '@site/src/types/chatkit.d';

/**
 * Utility class for managing conversation history persistence
 */
export class ConversationManager {
  private storageKey: string;

  constructor(storageKey: string = STORAGE_KEYS.CONVERSATION_THREAD) {
    this.storageKey = storageKey;
  }

  /**
   * Save a conversation thread to localStorage
   * @param thread - Conversation thread to persist
   * @returns true if save was successful, false otherwise
   */
  save(thread: ConversationThread): boolean {
    try {
      // Update lastUpdatedAt timestamp before saving
      const updatedThread: ConversationThread = {
        ...thread,
        lastUpdatedAt: new Date().toISOString(),
      };

      localStorage.setItem(this.storageKey, JSON.stringify(updatedThread));
      return true;
    } catch (error) {
      console.error('Failed to save conversation:', error);

      // Handle quota exceeded error
      if (error instanceof DOMException && error.name === 'QuotaExceededError') {
        console.warn('localStorage quota exceeded. Consider clearing old conversations.');

        // Try to save a reduced version (keep only recent messages)
        try {
          const reducedThread: ConversationThread = {
            ...thread,
            messages: thread.messages.slice(-50), // Keep last 50 messages
            lastUpdatedAt: new Date().toISOString(),
          };
          localStorage.setItem(this.storageKey, JSON.stringify(reducedThread));
          console.info('Saved reduced conversation (last 50 messages)');
          return true;
        } catch (retryError) {
          console.error('Failed to save reduced conversation:', retryError);
          return false;
        }
      }

      return false;
    }
  }

  /**
   * Load conversation thread from localStorage
   * @returns The conversation thread, or null if not found or invalid
   */
  load(): ConversationThread | null {
    try {
      const data = localStorage.getItem(this.storageKey);
      if (!data) {
        return null;
      }

      const thread: ConversationThread = JSON.parse(data);

      // Validate thread structure
      if (!this.isValidThread(thread)) {
        console.warn('Invalid conversation thread structure, clearing storage');
        this.clear();
        return null;
      }

      return thread;
    } catch (error) {
      console.error('Failed to load conversation:', error);
      return null;
    }
  }

  /**
   * Clear conversation from localStorage
   * @returns true if clear was successful, false otherwise
   */
  clear(): boolean {
    try {
      localStorage.removeItem(this.storageKey);
      return true;
    } catch (error) {
      console.error('Failed to clear conversation:', error);
      return false;
    }
  }

  /**
   * Check if a conversation exists in localStorage
   * @returns true if conversation exists, false otherwise
   */
  exists(): boolean {
    try {
      const data = localStorage.getItem(this.storageKey);
      return data !== null && data !== undefined;
    } catch (error) {
      console.error('Failed to check conversation existence:', error);
      return false;
    }
  }

  /**
   * Add a message to the existing conversation thread
   * Creates a new thread if none exists
   * @param message - Message to add
   * @param userId - User ID for the conversation
   * @returns The updated conversation thread
   */
  addMessage(message: Message, userId: string): ConversationThread {
    let thread = this.load();

    if (!thread) {
      // Create new thread if none exists
      thread = this.createThread(userId);
    }

    thread.messages.push(message);
    thread.lastUpdatedAt = new Date().toISOString();

    this.save(thread);
    return thread;
  }

  /**
   * Get message count for the current conversation
   * @returns Number of messages in the conversation
   */
  getMessageCount(): number {
    const thread = this.load();
    return thread ? thread.messages.length : 0;
  }

  /**
   * Get the last N messages from the conversation
   * @param count - Number of recent messages to retrieve
   * @returns Array of recent messages
   */
  getRecentMessages(count: number): Message[] {
    const thread = this.load();
    if (!thread) {
      return [];
    }

    return thread.messages.slice(-count);
  }

  /**
   * Get estimated storage size in bytes
   * @returns Approximate size of stored conversation in bytes
   */
  getStorageSize(): number {
    try {
      const data = localStorage.getItem(this.storageKey);
      if (!data) {
        return 0;
      }

      // Approximate size calculation (UTF-16 encoding)
      return new Blob([data]).size;
    } catch (error) {
      console.error('Failed to calculate storage size:', error);
      return 0;
    }
  }

  /**
   * Create a new conversation thread
   * @param userId - User ID for the new thread
   * @returns New conversation thread
   */
  private createThread(userId: string): ConversationThread {
    const now = new Date().toISOString();
    return {
      threadId: this.generateThreadId(),
      userId,
      messages: [],
      createdAt: now,
      lastUpdatedAt: now,
      status: 'active',
    };
  }

  /**
   * Validate conversation thread structure
   * @param thread - Thread to validate
   * @returns true if thread has valid structure
   */
  private isValidThread(thread: any): thread is ConversationThread {
    return (
      typeof thread === 'object' &&
      thread !== null &&
      typeof thread.threadId === 'string' &&
      typeof thread.userId === 'string' &&
      Array.isArray(thread.messages) &&
      typeof thread.createdAt === 'string' &&
      typeof thread.lastUpdatedAt === 'string' &&
      (thread.status === 'active' || thread.status === 'archived')
    );
  }

  /**
   * Generate a unique thread ID
   * @returns UUID-like string
   */
  private generateThreadId(): string {
    return `thread-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

// Export singleton instance for convenience
export const conversationManager = new ConversationManager();
