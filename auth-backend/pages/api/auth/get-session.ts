import type { NextApiRequest, NextApiResponse } from 'next';
import { applySecurityHeaders } from '../../../lib/security';
import { withCors } from '../../../middleware/cors';
import db from '../../../lib/database';
import { UserModel } from '../../../lib/user';

interface SessionResponse {
  user: {
    id: string;
    email: string;
    firstName: string;
    lastName: string;
    emailVerified: boolean;
    isActive: boolean;
  };
  session: {
    id: string;
    expiresAt: string;
  };
}

async function handler(
  req: NextApiRequest,
  res: NextApiResponse<SessionResponse | { error: string; message: string }>
) {
  // Apply security headers
  applySecurityHeaders(req, res);

  if (req.method !== 'GET') {
    return res.status(405).json({
      error: 'Method not allowed',
      message: 'Only GET requests are allowed',
    });
  }

  try {
    // 1. Extract Session Token from Authorization Header or Cookie
    let token: string | null = null;

    // A. Priority: Check Authorization Header (Bearer Token)
    if (req.headers.authorization) {
      const authHeader = req.headers.authorization;
      if (authHeader.startsWith('Bearer ')) {
        token = authHeader.substring(7);
      }
    }

    // B. Fallback: Check Cookie
    if (!token) {
      const cookieHeader = req.headers.cookie;
      if (cookieHeader && cookieHeader.includes('better-auth.session_token')) {
        token = cookieHeader.split('better-auth.session_token=')[1].split(';')[0].trim();
      }
    }

    // If no token found
    if (!token) {
      return res.status(401).json({
        error: 'Unauthorized',
        message: 'No session token found. Please log in.',
      });
    }

    // 2. Query database for session
    const result = await db.query(
      `SELECT * FROM "session" WHERE "id" = $1`,
      [token]
    );

    const session = result.rows[0];

    // If no session found
    if (!session) {
      return res.status(401).json({
        error: 'Unauthorized',
        message: 'Invalid or expired session.',
      });
    }

    // 3. Check if session is expired
    if (new Date(session.expiresAt) < new Date()) {
      return res.status(401).json({
        error: 'Unauthorized',
        message: 'Session expired. Please log in again.',
      });
    }

    // 4. Get user details
    const userId = session.userId || session.user_id;
    const user = await UserModel.findById(userId);

    if (!user) {
      return res.status(401).json({
        error: 'Unauthorized',
        message: 'User not found.',
      });
    }

    // 5. Check if user account is active
    if (!user.is_active) {
      return res.status(403).json({
        error: 'Forbidden',
        message: 'Account is deactivated.',
      });
    }

    // 6. Return session data
    return res.status(200).json({
      user: {
        id: user.id,
        email: user.email,
        firstName: user.first_name,
        lastName: user.last_name,
        emailVerified: user.email_verified,
        isActive: user.is_active,
      },
      session: {
        id: session.id,
        expiresAt: session.expiresAt,
      },
    });
  } catch (error: unknown) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    console.error('Get session error:', error);

    return res.status(500).json({
      error: 'Internal server error',
      message: 'Failed to validate session.',
    });
  }
}

export default withCors(handler);
