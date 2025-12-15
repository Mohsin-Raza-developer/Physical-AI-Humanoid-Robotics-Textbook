import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse, ProfileResponse } from '../../../types/auth';
import { UserModel } from '../../../lib/user';

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

  if (req.method !== 'GET') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // In a real implementation, we would extract the user ID from the session/token
    // For now, I'll pass userId as a query parameter for testing purposes
    // In production, this should be validated against the user's session
    const userId = req.query.userId as string;

    if (!userId) {
      return res.status(401).json({
        success: false,
        message: 'Authentication required',
      });
    }

    const user = await UserModel.findById(userId);

    if (!user) {
      return res.status(404).json({
        success: false,
        message: 'User not found',
      });
    }

    const response: ProfileResponse = {
      success: true,
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