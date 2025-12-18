import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse } from '../../../types/auth';
import { UserModel } from '../../../lib/user';
import { authRateLimit } from '../../../lib/rateLimit';
import { authMetrics } from '../../../lib/logger';
import { applySecurityHeaders } from '../../../lib/security';
import { withAuth, AuthenticatedRequest } from '../../../middleware/auth';
import { withCors } from '../../../middleware/cors';

async function handler(
  req: AuthenticatedRequest,
  res: NextApiResponse<AuthResponse>
) {
  // Apply security headers
  applySecurityHeaders(req, res);

  // Note: Rate limiting not needed here as route is already protected by withAuth middleware

  if (req.method !== 'DELETE') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Get userId securely from the authenticated session
    // (Injected by withAuth middleware)
    const userId = req.user?.id;

    if (!userId) {
      return res.status(401).json({
        success: false,
        message: 'Authentication required - User ID missing from session',
      });
    }

    // Find the user to verify existence before deletion
    const user = await UserModel.findById(userId);

    if (!user) {
      return res.status(404).json({
        success: false,
        message: 'User not found',
      });
    }

    // Perform GDPR-compliant account deletion (hard delete)
    const deletionResult = await UserModel.hardDeleteUser(userId);

    if (!deletionResult) {
      // If 0 rows were deleted, check if the user is effectively gone
      const userCheck = await UserModel.findById(userId);
      if (!userCheck) {
        return res.status(200).json({
          success: true,
          message: 'Account deleted successfully (already removed)',
        });
      }

      return res.status(500).json({
        success: false,
        message: 'Failed to delete account',
      });
    }

    // Log the successful deletion (optional but good for audit)
    // authLogger.accountDeletion(userId); 

    return res.status(200).json({
      success: true,
      message: 'Account deleted successfully',
    });
  } catch (error: any) {
    console.error('Account deletion error:', error);

    return res.status(500).json({
      success: false,
      message: error.message || 'Internal server error',
    });
  }
}

// Wrap the handler with CORS and Auth middleware
export default withCors(withAuth(handler));