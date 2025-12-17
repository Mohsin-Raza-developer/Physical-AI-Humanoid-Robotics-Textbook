import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, ProfileResponse } from '../../../types/auth';
import { UserModel } from '../../../lib/user';
import { withAuth, AuthenticatedRequest } from '../../../middleware/auth';
import { withCors } from '../../../middleware/cors';

async function handler(
  req: AuthenticatedRequest,
  res: NextApiResponse<AuthResponse>
) {
  // Get User ID from secure session (injected by withAuth)
  const userId = req.user?.id;

  if (!userId) {
    return res.status(401).json({
      success: false,
      message: 'Authentication required - User ID missing',
    });
  }

  // Handle Profile Update (PUT)
  if (req.method === 'PUT') {
    try {
      // Check if user exists first
      const existingUser = await UserModel.findById(userId);
      if (!existingUser) {
        return res.status(404).json({
          success: false,
          message: 'User not found',
        });
      }

      const updateData = {
        software_level: req.body.software_level,
        hardware_access: req.body.hardware_access,
      };

      const updatedUser = await UserModel.updateProfile(userId, updateData);

      if (!updatedUser) {
        return res.status(500).json({
          success: false,
          message: 'Failed to update profile',
        });
      }

      return res.status(200).json({
        success: true,
        message: 'Profile updated successfully',
        data: {
          id: updatedUser.id,
          email: updatedUser.email,
          first_name: updatedUser.first_name,
          last_name: updatedUser.last_name,
          software_level: updatedUser.software_level,
          hardware_access: updatedUser.hardware_access,
          created_at: updatedUser.created_at,
          last_login_at: updatedUser.last_login_at || undefined,
          email_verified: updatedUser.email_verified,
          verification_token: updatedUser.verification_token || undefined,
          is_active: updatedUser.is_active,
          updated_at: updatedUser.updated_at,
        }
      });
    } catch (error: any) {
      console.error('Update profile error:', error);
      return res.status(500).json({
        success: false,
        message: 'Internal server error',
        error: process.env.NODE_ENV === 'development' ? error.message : undefined
      });
    }
  }

  // Handle Profile Fetch (GET)
  if (req.method === 'GET') {
    try {
      const user = await UserModel.findById(userId);

      if (!user) {
        return res.status(404).json({
          success: false,
          message: 'User not found',
        });
      }

      const response: ProfileResponse = {
        success: true,
        message: 'Profile retrieved successfully',
        data: {
          id: user.id,
          email: user.email,
          first_name: user.first_name,
          last_name: user.last_name,
          software_level: user.software_level,
          hardware_access: user.hardware_access,
          created_at: user.created_at,
          last_login_at: user.last_login_at || undefined,
          email_verified: user.email_verified,
          verification_token: user.verification_token || undefined,
          is_active: user.is_active,
          updated_at: user.updated_at,
        }
      };

      return res.status(200).json(response);
    } catch (error: any) {
      console.error('Get profile error:', error);

      return res.status(500).json({
        success: false,
        message: 'Internal server error',
      });
    }
  }

  return res.status(405).json({
    success: false,
    message: 'Method not allowed',
  });
}

export default withCors(withAuth(handler));