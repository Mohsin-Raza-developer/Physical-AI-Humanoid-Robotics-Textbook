import { validateNewUser, sanitizeInput } from '../../../lib/validation';
import { UserModel } from '../../../lib/user';
import bcrypt from 'bcryptjs';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, RegistrationResponse, RegistrationRequest } from '../../../types/auth';
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
      process.env.CORS_ORIGIN || 'http://localhost:3003',
      'http://localhost:3000',
      'http://localhost:3001',
      'http://localhost:3002',
      'http://localhost:3003',
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
    authMetrics.rateLimitHit('register');
    return; // Rate limit response already sent
  }

  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Track registration attempt
    authMetrics.registerAttempt();

    // Parse and sanitize request body
    const { email, password, firstName, lastName, softwareLevel, hardwareAccess }: RegistrationRequest = sanitizeInput(req.body);

    // Get IP and user agent for logging
    const ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
    const userAgent = req.headers['user-agent'];

    // Log registration attempt
    authLogger.register(undefined, ip?.toString(), userAgent, { email });

    // Validate the input data
    const validation = validateNewUser({
      email,
      password,
      firstName,
      lastName,
      softwareLevel,
      hardwareAccess,
    });

    if (!validation.isValid) {
      authMetrics.registerFailure();
      authLogger.error('register-validation', undefined, ip?.toString(), userAgent, { email, errors: validation.errors });

      return res.status(400).json({
        success: false,
        message: 'Validation failed',
        errors: validation.errors.map(error => ({
          field: 'validation',
          message: error
        }))
      });
    }

    // Check if user already exists
    const existingUser = await UserModel.findByEmail(email);
    if (existingUser) {
      authMetrics.registerFailure();
      authLogger.error('register-duplicate-email', undefined, ip?.toString(), userAgent, { email });

      return res.status(409).json({
        success: false,
        message: 'Email already registered',
      });
    }

    // Hash the password
    const saltRounds = 12;
    const password_hash = await bcrypt.hash(password, saltRounds);

    // Create the new user with the hashed password
    const newUser = await UserModel.create({
      email,
      first_name: firstName,
      last_name: lastName,
      password_hash,
      software_level: softwareLevel,
      hardware_access: hardwareAccess,
    });

    // Track successful registration
    authMetrics.registerSuccess();

    // Log successful registration
    // Log successful registration
    authLogger.register(newUser.id, ip?.toString(), userAgent, {
      email: newUser.email,
      firstName: newUser.first_name,
      lastName: newUser.last_name,
      softwareLevel: newUser.software_level,
      hardwareAccess: newUser.hardware_access
    });

    // Return success response
    const response: RegistrationResponse = {
      success: true,
      message: 'User registered successfully',
      data: {
        userId: newUser.id,
        email: newUser.email,
        createdAt: newUser.created_at.toISOString(),
      }
    };

    return res.status(201).json(response);
  } catch (error: any) {
    authMetrics.registerFailure();
    authLogger.error('register', undefined, undefined, undefined, { error: error.message });

    console.error('Registration error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}