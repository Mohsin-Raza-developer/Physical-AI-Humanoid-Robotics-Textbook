import { validateEmail, sanitizeInput } from '../../../../lib/validation';
import { UserModel } from '../../../../lib/user';
import { PasswordResetTokenModel } from '../../../../lib/passwordResetToken';
import { sendPasswordResetEmail } from '../../../../lib/email';
import { authMetrics } from '../../../../lib/logger';
import crypto from 'crypto';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, PasswordResetRequest } from '../../../../types/auth';
import { authRateLimit } from '../../../../lib/rateLimit';
import { applySecurityHeaders } from '../../../../lib/security';
import { withCors } from '../../../../middleware/cors';

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
    const { email }: PasswordResetRequest = sanitizeInput(req.body);

    // Apply email-based rate limiting for password reset requests
    if (!(await authRateLimit(req, res, email))) {
      authMetrics.rateLimitHit('reset-password-request');
      return; // Rate limit response already sent
    }

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

export default withCors(handler);