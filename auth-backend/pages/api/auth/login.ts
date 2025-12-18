import { validateLogin, sanitizeInput } from '../../../lib/validation';
import { UserModel } from '../../../lib/user';
import bcrypt from 'bcryptjs';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, LoginResponse, LoginRequest } from '../../../types/auth';
import { authRateLimit } from '../../../lib/rateLimit';
import { applySecurityHeaders } from '../../../lib/security';
import { authLogger, authMetrics } from '../../../lib/logger';
import { withCors } from '../../../middleware/cors';
import crypto from 'crypto';
import db from '../../../lib/database';

async function handler(
  req: NextApiRequest,
  res: NextApiResponse<AuthResponse>
) {
  // Apply security headers
  applySecurityHeaders(req, res);

  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Parse and sanitize request body first to get email
    const { email, password }: LoginRequest = sanitizeInput(req.body);

    // Apply email-based rate limiting for login attempts
    if (!(await authRateLimit(req, res, email))) {
      authMetrics.rateLimitHit('login');
      return; // Rate limit response already sent
    }

    // Get IP and user agent for logging
    const ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
    const userAgent = req.headers['user-agent'];

    // Track login attempt
    authMetrics.loginAttempt();

    // Validate the input data
    const validation = validateLogin({ email, password });

    if (!validation.isValid) {
      authMetrics.loginFailure();
      authLogger.error('login-validation', undefined, ip?.toString(), userAgent, { email, errors: validation.errors });

      return res.status(400).json({
        success: false,
        message: 'Validation failed',
        errors: validation.errors.map(error => ({
          field: 'validation',
          message: error
        }))
      });
    }

    // Find the user by email
    const user = await UserModel.findByEmail(email);

    if (!user) {
      authMetrics.loginFailure();
      // Don't reveal that the user doesn't exist for security reasons, but log it internally
      authLogger.error('login-user-not-found', undefined, ip?.toString(), userAgent, { email });

      return res.status(401).json({
        success: false,
        message: 'Invalid email or password',
      });
    }

    // Check if password matches
    const isPasswordValid = await bcrypt.compare(password, user.password_hash);

    if (!isPasswordValid) {
      authMetrics.loginFailure();
      authLogger.error('login-failed', user.id, ip?.toString(), userAgent, { email });

      return res.status(401).json({
        success: false,
        message: 'Invalid email or password',
      });
    }

    // Check if account is active
    if (!user.is_active) {
      authMetrics.loginFailure();
      authLogger.error('login-inactive', user.id, ip?.toString(), userAgent, { email });

      return res.status(403).json({
        success: false,
        message: 'Account is deactivated. Please contact support.',
      });
    }

    // Update last login time
    await UserModel.updateLastLogin(user.id);

    // Track successful login
    authMetrics.loginSuccess();

    // Log successful login
    authLogger.login(user.id, ip?.toString(), userAgent, {
      email: user.email,
      userId: user.id
    });

    // --- MANUAL SESSION CREATION ---
    // Generate a secure token
    const token = crypto.randomBytes(32).toString('hex');
    const expiresAt = new Date(Date.now() + 30 * 24 * 60 * 60 * 1000); // 30 days

    // Insert session into database (using our 'session' table)
    console.log('[Login] Inserting Session:', token);
    const insertResult = await db.query(
      `INSERT INTO "session" ("id", "userId", "expiresAt", "ipAddress", "userAgent")
       VALUES ($1, $2, $3, $4, $5)`,
      [token, user.id, expiresAt, ip?.toString() || null, userAgent || null]
    );
    console.log('[Login] Session Inserted via db.query. RowCount:', insertResult.rowCount);

    // Set the session cookie
    const isProduction = process.env.NODE_ENV === 'production';
    const cookieName = 'better-auth.session_token';
    const maxAge = 30 * 24 * 60 * 60; // 30 days

    // Clear old cookie AND Set new cookie (Force refresh)
    const clearCookie = `${cookieName}=; Path=/; HttpOnly; Max-Age=0`;

    // Cookie Header Construction
    // IMPORTANT: On localhost (http), Secure MUST be omitted.
    let cookieValue = `${cookieName}=${token}; Path=/; HttpOnly; Max-Age=${maxAge}`;

    if (isProduction) {
      cookieValue += '; Secure';
    }

    // Send array of cookies
    res.setHeader('Set-Cookie', [clearCookie, cookieValue]);
    // --------------------------------

    const response: LoginResponse = {
      success: true,
      message: 'Login successful',
      data: {
        userId: user.id,
        email: user.email,
        firstName: user.first_name,
        lastName: user.last_name,
        sessionTimeout: 2592000000,
        token: token // Sending token in body as well just in case
      }
    };

    return res.status(200).json(response);
  } catch (error: unknown) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    authMetrics.loginFailure();
    authLogger.error('login', undefined, undefined, undefined, { error: errorMessage });

    console.error('Login error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}

export default withCors(handler);