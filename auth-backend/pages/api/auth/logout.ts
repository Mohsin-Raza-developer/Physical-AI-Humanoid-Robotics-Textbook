import type { NextApiRequest, NextApiResponse } from 'next';
import type { AuthResponse } from '../../../types/auth';
import { withCors } from '../../../middleware/cors';

async function handler(
  req: NextApiRequest,
  res: NextApiResponse<AuthResponse>
) {
  // Note: We don't apply rate limiting to logout as it's generally safe
  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      message: 'Method not allowed',
    });
  }

  try {
    // Clear the session cookie
    res.setHeader('Set-Cookie', 'better-auth.session_token=; Path=/; HttpOnly; Max-Age=0; Expires=Thu, 01 Jan 1970 00:00:00 GMT');

    return res.status(200).json({
      success: true,
      message: 'Logged out successfully',
    });
  } catch (error: any) {
    console.error('Logout error:', error);

    return res.status(500).json({
      success: false,
      message: 'Internal server error',
    });
  }
}

export default withCors(handler);