import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse } from '../../../types/auth';
import { UserModel } from '../../../lib/user';

import { authRateLimit } from '../../../lib/rateLimit';
import { authMetrics } from '../../../lib/logger';
import { applySecurityHeaders } from '../../../lib/security';

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

  // Apply rate limiting for account deletion endpoint
  if (!(await authRateLimit(req, res))) {
    authMetrics.rateLimitHit('account-deletion');
    return; // Rate limit response already sent
  }

  if (req.method !== 'DELETE') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // In a real implementation, we would validate the user's authentication here
    // For now, we'll accept the userId as part of the request but in real app this 
    // should be extracted from the authenticated session
    const userId = req.body.userId || req.query.userId;

    if (!userId) {
      return res.status(401).json({
        success: false,
        message: 'Authentication required',
      });
    }

    // Find the user to get their email for notification
    const user = await UserModel.findById(userId);

    if (!user) {
      return res.status(404).json({
        success: false,
        message: 'User not found',
      });
    }

    // Perform GDPR-compliant account deletion (hard delete to completely remove personal data)
    const deletionResult = await UserModel.hardDeleteUser(userId);

    if (!deletionResult) {
      return res.status(500).json({
        success: false,
        message: 'Failed to delete account',
      });
    }

    // Email notification for deletion removed as per request

    return res.status(200).json({
      success: true,
      message: 'Account deleted successfully',
    });
  } catch (error: any) {
    console.error('Account deletion error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}