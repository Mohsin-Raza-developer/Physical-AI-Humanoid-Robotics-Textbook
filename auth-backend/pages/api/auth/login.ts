import type { NextApiRequest, NextApiResponse } from 'next';
import { UserModel } from '../../../lib/user';
import bcrypt from 'bcryptjs';
import type { AuthResponse, LoginResponse, LoginRequest } from '../../../types/auth';
import { validateEmail, sanitizeInput } from '../../../lib/validation';
import { authRateLimit } from '../../../lib/rateLimit';
import { applySecurityHeaders } from '../../../lib/security';
import { authLogger, authMetrics } from '../../../lib/logger';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<AuthResponse>
) {
  // Handle CORS preflight requests first
  if (req.method === 'OPTIONS') {
    const origin = req.headers.origin;
    const allowedOrigins = [
      process.env.CORS_ORIGIN || 'http://localhost:3000',
      'http://localhost:3000',
      'http://localhost:3001',
      'http://localhost:3002',
      'https://physical-ai-humanoid-robotics-textbook.github.io'
    ];

    if (origin && allowedOrigins.includes(origin)) {
      res.setHeader('Access-Control-Allow-Origin', origin);
    } else {
      res.setHeader('Access-Control-Allow-Origin', allowedOrigins[0]);
    }

    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization, Content-Length, Cache-Control');
    res.setHeader('Access-Control-Allow-Credentials', 'true');

    res.status(200).end();
    return;
  }

  // Apply security headers
  applySecurityHeaders(req, res);

  // Apply rate limiting for authentication endpoints
  if (!(await authRateLimit(req, res))) {
    authMetrics.rateLimitHit('login');
    return; // Rate limit response already sent
  }

  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Track login attempt
    authMetrics.loginAttempt();

    // Parse and sanitize request body
    const { email, password }: LoginRequest = sanitizeInput(req.body);

    // Get IP and user agent for logging
    const ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
    const userAgent = req.headers['user-agent'];

    // Log login attempt
    authLogger.login(undefined, ip?.toString(), userAgent, { email });

    // Validate email format
    const emailValidation = validateEmail(email);
    if (!emailValidation.isValid) {
      authMetrics.loginFailure();
      authLogger.error('login-validation', undefined, ip?.toString(), userAgent, { email, error: emailValidation.errors[0] || 'Invalid email format' });

      return res.status(400).json({
        success: false,
        message: 'Validation failed',
        errors: [
          {
            field: 'email',
            message: emailValidation.errors[0] || 'Invalid email format'
          }
        ]
      });
    }

    // Check if user exists
    const user = await UserModel.findByEmail(email);
    if (!user) {
      authMetrics.loginFailure();
      authLogger.error('login-credentials', undefined, ip?.toString(), userAgent, { email, error: 'Invalid email or password' });

      return res.status(401).json({
        success: false,
        message: 'Invalid email or password',
      });
    }

    // Check if password is correct
    const isPasswordValid = await bcrypt.compare(password, user.password_hash);
    if (!isPasswordValid) {
      authMetrics.loginFailure();
      authLogger.error('login-credentials', user.id, ip?.toString(), userAgent, { email, error: 'Invalid email or password' });

      return res.status(401).json({
        success: false,
        message: 'Invalid email or password',
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

    // In a real implementation with Better Auth, we would create a session here
    // For now, we'll return a response indicating successful login
    const response: LoginResponse = {
      success: true,
      message: 'Login successful',
      data: {
        userId: user.id,
        email: user.email,
        firstName: user.first_name,
        lastName: user.last_name,
        // 30 days in milliseconds (30 * 24 * 60 * 60 * 1000 = 2,592,000,000 ms)
        sessionTimeout: 2592000000,
      }
    };

    return res.status(200).json(response);
  } catch (error: any) {
    authMetrics.loginFailure();
    authLogger.error('login', undefined, undefined, undefined, { error: error.message });

    console.error('Login error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}