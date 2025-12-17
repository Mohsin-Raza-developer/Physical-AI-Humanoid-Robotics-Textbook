import db from './database';

export interface User {
  id: string;
  email: string;
  first_name: string;
  last_name: string;
  password_hash: string;
  software_level: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
  created_at: Date;
  updated_at: Date;
  last_login_at?: Date | null;
  email_verified: boolean;
  verification_token?: string | null;
  is_active: boolean;
}

export interface CreateUserInput {
  email: string;
  first_name: string;
  last_name: string;
  password_hash: string;
  software_level: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
  email_verified?: boolean;
  verification_token?: string | null;
}

export interface UpdateUserProfileInput {
  software_level?: 'Beginner' | 'Intermediate' | 'Advanced';
  hardware_access?: 'Laptop/Cloud' | 'RTX GPU' | 'Jetson Edge' | 'Physical Robot';
}

export class UserModel {
  static tableName = 'users';

  /**
   * Creates a new user in the database
   */
  static async create(userData: CreateUserInput): Promise<User> {
    const { email, first_name, last_name, password_hash, software_level, hardware_access, email_verified = false, verification_token } = userData;

    const query = `
      INSERT INTO ${this.tableName} 
      (email, first_name, last_name, password_hash, software_level, hardware_access, email_verified, verification_token)
      VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
      RETURNING *
    `;

    const result = await db.query(query, [
      email,
      first_name,
      last_name,
      password_hash,
      software_level,
      hardware_access,
      email_verified,
      verification_token
    ]);

    return result.rows[0];
  }

  /**
   * Finds a user by their email
   */
  static async findByEmail(email: string): Promise<User | null> {
    const query = `SELECT * FROM ${this.tableName} WHERE email = $1 AND is_active = true`;

    const result = await db.query(query, [email]);

    return result.rows[0] || null;
  }

  /**
   * Finds a user by their ID
   */
  static async findById(id: string): Promise<User | null> {
    const query = `SELECT * FROM ${this.tableName} WHERE id = $1 AND is_active = true`;

    const result = await db.query(query, [id]);

    return result.rows[0] || null;
  }

  /**
   * Updates a user's profile information
   */
  static async updateProfile(userId: string, profileData: UpdateUserProfileInput): Promise<User | null> {
    const { software_level, hardware_access } = profileData;

    const updateFields: string[] = [];
    const values: any[] = [];
    let paramIndex = 1;

    if (software_level !== undefined) {
      updateFields.push(`software_level = $${paramIndex}`);
      values.push(software_level);
      paramIndex++;
    }
    if (hardware_access !== undefined) {
      updateFields.push(`hardware_access = $${paramIndex}`);
      values.push(hardware_access);
      paramIndex++;
    }

    // Add updated_at as the final parameter
    updateFields.push(`updated_at = NOW()`);
    values.push(userId);

    const query = `
      UPDATE ${this.tableName}
      SET ${updateFields.join(', ')}
      WHERE id = $${paramIndex} AND is_active = true
      RETURNING *
    `;

    const result = await db.query(query, values);

    return result.rows[0] || null;
  }

  /**
   * Updates the last login time for a user
   */
  static async updateLastLogin(userId: string): Promise<void> {
    const query = `
      UPDATE ${this.tableName}
      SET last_login_at = NOW()
      WHERE id = $1
    `;

    await db.query(query, [userId]);
  }

  /**
   * Sets email verification status
   */
  static async setEmailVerified(userId: string, verified: boolean = true): Promise<User | null> {
    const query = `
      UPDATE ${this.tableName}
      SET email_verified = $1, verification_token = NULL
      WHERE id = $2 AND is_active = true
      RETURNING *
    `;

    const result = await db.query(query, [verified, userId]);

    return result.rows[0] || null;
  }

  /**
   * Verifies an email using a token
   */
  static async verifyEmailByToken(token: string): Promise<User | null> {
    const query = `
      UPDATE ${this.tableName}
      SET email_verified = true, verification_token = NULL
      WHERE verification_token = $1 AND is_active = true
      RETURNING *
    `;

    const result = await db.query(query, [token]);

    return result.rows[0] || null;
  }

  /**
   * Deletes a user account (soft delete for GDPR compliance)
   */
  static async deleteUser(userId: string): Promise<boolean> {
    const query = `
      UPDATE ${this.tableName}
      SET is_active = false, email_verified = false, verification_token = NULL
      WHERE id = $1
      RETURNING id
    `;

    const result = await db.query(query, [userId]);

    return result.rowCount !== 0;
  }

  /**
   * Sets a verification token for a user
   */
  static async setVerificationToken(userId: string, token: string): Promise<User | null> {
    const query = `
      UPDATE ${this.tableName}
      SET verification_token = $1
      WHERE id = $2 AND is_active = true
      RETURNING *
    `;

    const result = await db.query(query, [token, userId]);

    return result.rows[0] || null;
  }

  /**
   * Updates a user's password
   */
  static async updatePassword(userId: string, newPasswordHash: string): Promise<User | null> {
    const query = `
      UPDATE ${this.tableName}
      SET password_hash = $1, updated_at = NOW()
      WHERE id = $2 AND is_active = true
      RETURNING *
    `;

    const result = await db.query(query, [newPasswordHash, userId]);

    return result.rows[0] || null;
  }

  /**
   * Completely removes a user account and all related data for GDPR compliance
   * This is a hard delete that permanently removes the user and related data
   */
  static async hardDeleteUser(userId: string): Promise<boolean> {
    try {
      // First delete related data (password reset tokens, etc.)
      await this.deleteRelatedData(userId);

      // Then delete the user record
      const query = `
        DELETE FROM ${this.tableName}
        WHERE id = $1
      `;

      const result = await db.query(query, [userId]);

      return result.rowCount !== 0;
    } catch (error) {
      console.error('Error during hard delete user:', error);
      throw error;
    }
  }

  /**
   * Helper function to delete related user data (password reset tokens, etc.)
   */
  private static async deleteRelatedData(userId: string): Promise<void> {
    // Import PasswordResetTokenModel if needed
    // Delete password reset tokens for this user
    // Delete password reset tokens for this user
    try {
      const deletePasswordResetTokensQuery = `
        DELETE FROM password_reset_tokens
        WHERE user_id = $1
      `;
      await db.query(deletePasswordResetTokensQuery, [userId]);
    } catch (err) {
      // Ignore if table doesn't exist or other error, just verify logs
      console.warn('Could not delete related password tokens (non-fatal):', err);
    }
  }
}