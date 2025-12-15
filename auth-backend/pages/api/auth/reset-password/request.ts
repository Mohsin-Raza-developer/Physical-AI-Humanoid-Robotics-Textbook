import { validateEmail, sanitizeInput } from '../../../../lib/validation';
import { UserModel } from '../../../../lib/user';
import { PasswordResetTokenModel } from '../../../../lib/passwordResetToken';
import { sendPasswordResetEmail } from '../../../../lib/email';
import crypto from 'crypto';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, PasswordResetRequest } from '../../../../types/auth';
import { authRateLimit } from '../../../../lib/rateLimit';
import { applySecurityHeaders } from '../../../../lib/security';

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
    authMetrics.rateLimitHit('reset-password-request');
    return; // Rate limit response already sent
  }

  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Parse and sanitize request body
    const { email }: PasswordResetRequest = sanitizeInput(req.body);

    // Validate email format
    const emailValidation = validateEmail(email);
    if (!emailValidation.isValid) {
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

    // Find the user by email
    const user = await UserModel.findByEmail(email);

    // Don't reveal if the email exists or not for security reasons
    if (!user) {
      // Still return success to prevent user enumeration
      return res.status(200).json({
        success: true,
        message: 'Password reset email sent if email exists'
      });
    }

    // Generate a secure token
    const token = crypto.randomBytes(32).toString('hex');

    // Create the password reset token in the database
    await PasswordResetTokenModel.create({
      userId: user.id,
      token,
      // Token expires in 1 hour (3600 seconds)
      expiresAt: new Date(Date.now() + 3600000), // 1 hour from now
    });

    // Generate reset URL (Pointing to Docusaurus Frontend)
    const siteUrl = process.env.NEXT_PUBLIC_SITE_URL || 'http://localhost:3000/Physical-AI-Humanoid-Robotics-Textbook';
    const resetUrl = `${siteUrl}/auth/reset-password?token=${encodeURIComponent(token)}`;

    console.log('\n=================================================================');
    console.log('🔑 PASSWORD RESET LINK (Test Mode):');
    console.log(resetUrl);
    console.log('=================================================================\n');

    // Send password reset email
    await sendPasswordResetEmail({ email: user.email, resetToken: token, userName: user.email.split('@')[0] });

    // Return success response (don't reveal if user exists)
    return res.status(200).json({
      success: true,
      message: 'Password reset email sent if email exists'
    });
  } catch (error: any) {
    console.error('Password reset request error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}