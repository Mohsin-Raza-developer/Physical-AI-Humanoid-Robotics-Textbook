import { PasswordResetTokenModel } from '../../../../lib/passwordResetToken';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, PasswordResetValidateResponse } from '../../../../types/auth';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<AuthResponse>
) {
  if (req.method !== 'GET') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Get token from query parameters
    const { token } = req.query;

    if (!token || typeof token !== 'string') {
      return res.status(400).json({
        success: false,
        valid: false,
        message: 'Token is required',
      });
    }

    // Find the token in the database
    const resetToken = await PasswordResetTokenModel.findByToken(token);

    if (!resetToken) {
      return res.status(400).json({
        success: false,
        valid: false,
        message: 'Invalid or expired token',
      });
    }

    // Check if the token is expired
    if (resetToken.expires_at < new Date()) {
      // Token is expired, clean it up from the database
      await PasswordResetTokenModel.delete(token);

      return res.status(400).json({
        success: false,
        valid: false,
        message: 'Invalid or expired token',
      });
    }

    // Token is valid and not expired
    const response: PasswordResetValidateResponse = {
      success: true,
      valid: true,
      expiresAt: resetToken.expires_at.toISOString(),
    };

    return res.status(200).json(response);
  } catch (error: any) {
    console.error('Password reset validation error:', error);

    return res.status(500).json({
      success: false,
      valid: false,
      message: 'Internal server error',
    });
  }
}