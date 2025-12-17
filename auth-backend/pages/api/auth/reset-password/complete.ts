import { validateNewPassword } from '../../../../lib/validation';
import { PasswordResetTokenModel } from '../../../../lib/passwordResetToken';
import { UserModel } from '../../../../lib/user';
import bcrypt from 'bcryptjs';
import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, PasswordResetCompleteRequest } from '../../../../types/auth';
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
    // Parse request body
    const { token, newPassword }: PasswordResetCompleteRequest = req.body;

    // Validate token exists
    if (!token) {
      return res.status(400).json({
        success: false,
        message: 'Token is required',
      });
    }

    // Validate new password
    const passwordValidation = validateNewPassword(newPassword);
    if (!passwordValidation.isValid) {
      return res.status(400).json({
        success: false,
        message: 'Validation failed',
        errors: passwordValidation.errors.map(error => ({
          field: 'newPassword',
          message: error
        }))
      });
    }

    // Find the token in the database
    const resetToken = await PasswordResetTokenModel.findByToken(token);

    if (!resetToken) {
      return res.status(400).json({
        success: false,
        message: 'Invalid or expired token',
      });
    }

    // Check if the token is expired
    if (resetToken.expires_at < new Date()) {
      // Token is expired, clean it up from the database
      await PasswordResetTokenModel.delete(token);

      return res.status(400).json({
        success: false,
        message: 'Invalid or expired token',
      });
    }

    // Hash the new password
    const saltRounds = 12;
    const newPasswordHash = await bcrypt.hash(newPassword, saltRounds);

    // Update the user's password
    await UserModel.updatePassword(resetToken.user_id, newPasswordHash);

    // Mark the token as used and delete it
    await PasswordResetTokenModel.delete(token);

    return res.status(200).json({
      success: true,
      message: 'Password reset successfully',
    });
  } catch (error: any) {
    console.error('Password reset complete error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}

export default withCors(handler);