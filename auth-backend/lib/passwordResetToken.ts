import db from './database';
import { User } from './user';

export interface PasswordResetToken {
  id: string;
  user_id: string;
  token: string;
  expires_at: Date;
  created_at: Date;
}

export class PasswordResetTokenModel {
  static tableName = 'password_reset_tokens';

  /**
   * Creates a new password reset token
   */
  static async create(params: { userId: string, token: string, expiresAt?: Date }): Promise<PasswordResetToken> {
    // Delete any existing tokens for this user to ensure only one valid token exists
    await this.deleteForUser(params.userId);

    const expiresAt = params.expiresAt || new Date(Date.now() + 3600000); // 1 hour from now

    const query = `
      INSERT INTO ${this.tableName} (user_id, token, expires_at)
      VALUES ($1, $2, $3)
      RETURNING *
    `;

    const result = await db.query(query, [params.userId, params.token, expiresAt]);

    return result.rows[0];
  }

  /**
   * Delete a token by its string value
   */
  static async delete(token: string): Promise<boolean> {
    const query = `
      DELETE FROM ${this.tableName}
      WHERE token = $1
    `;

    const result = await db.query(query, [token]);

    return result.rowCount !== 0;
  }

  /**
   * Finds a valid reset token by token string
   */
  static async findByToken(token: string): Promise<PasswordResetToken | null> {
    // Also proactively delete if expired? Or just filter.
    // Filtering is safer read-only. Cleanup happens elsewhere.
    const query = `
      SELECT * FROM ${this.tableName}
      WHERE token = $1 
        AND expires_at > NOW()
    `;

    const result = await db.query(query, [token]);

    return result.rows[0] || null;
  }

  /**
   * Helper to delete all tokens for a specific user
   */
  private static async deleteForUser(userId: string): Promise<void> {
    const query = `
      DELETE FROM ${this.tableName}
      WHERE user_id = $1
    `;

    await db.query(query, [userId]);
  }

  /**
   * Cleans up expired tokens (should be run periodically)
   */
  static async cleanupExpiredTokens(): Promise<number> {
    const query = `
      DELETE FROM ${this.tableName}
      WHERE expires_at <= NOW()
    `;

    const result = await db.query(query);

    return result.rowCount || 0;
  }
}