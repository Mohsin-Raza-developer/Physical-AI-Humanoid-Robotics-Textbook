import { NextApiHandler, NextApiRequest, NextApiResponse } from 'next';
import { User, UserModel } from '../lib/user';

export interface AuthenticatedRequest extends NextApiRequest {
  user?: any; // User object attached by the middleware
}

/**
 * Middleware to verify authentication for API endpoints
 * This middleware checks if the user is authenticated and attaches user info to the request
 */
import db from '../lib/database';

export const authenticateUser = async (req: AuthenticatedRequest, res: NextApiResponse, next: () => void) => {
  try {
    // 1. Extract Session Token from either Authorization Header or Cookie
    let token: string | null = null;

    // A. Priority: Check Authorization Header (Bearer Token) - This is more reliable for frontend requests
    if (req.headers.authorization) {
      const authHeader = req.headers.authorization;
      if (authHeader.startsWith('Bearer ')) {
        token = authHeader.substring(7);
        console.log('[Auth] Token extracted from Authorization header'); // DEBUG LOG
      }
    }

    // B. Fallback: Check Cookie if Authorization header is missing
    if (!token) {
      const cookieHeader = req.headers.cookie;
      console.log('[Auth] Cookie Header:', cookieHeader); // DEBUG LOG

      if (cookieHeader && cookieHeader.includes('better-auth.session_token')) {
        token = cookieHeader.split('better-auth.session_token=')[1].split(';')[0].trim();
        console.log('[Auth] Token extracted from cookie'); // DEBUG LOG
      }
    }

    // If no token found in either location, return 401
    if (!token) {
      console.log('[Auth] No token found in Authorization header or cookies');
      res.status(401).json({
        success: false,
        message: 'No session token found. Please log in.'
      });
      return;
    }

    // 2. Check Database for this Session Token
    console.log(`[Auth] Querying DB for token: '${token}' (Length: ${token.length})`);

    // First check if it exists AT ALL (ignore expiry)
    const result = await db.query(
      `SELECT * FROM "session" WHERE "id" = $1`,
      [token]
    );

    const session = result.rows[0];
    console.log('[Auth] DB Raw Result:', session ? 'Found' : 'Not Found');

    if (session) {
      console.log('[Auth] Validating Expiry:', session.expiresAt, 'vs Now:', new Date());
      if (new Date(session.expiresAt) < new Date()) {
        console.log('[Auth] Session Expired!');
        // Explicitly fail if expired
        res.status(401).json({ success: false, message: 'Session expired' });
        return;
      }
    }

    // If no valid session found
    if (!session) {
      res.status(401).json({
        success: false,
        message: 'Invalid or expired session.'
      });
      return;
    }

    // 3. Attach User Info (Standardize req.user)
    // IMPORTANT: Spread session FIRST, then overwrite 'id' with userId.
    // Otherwise session.id (which is the token) overwrites user.id.
    req.user = {
      ...session,
      authenticated: true,
      id: session.userId || session.user_id, // Ensure this is the User UUID
      sessionId: session.id // Keep token as sessionId if needed
    };

    // Proceed to the next middleware or route handler
    next();
  } catch (error) {
    console.error('Authentication middleware error:', error);
    res.status(500).json({
      success: false,
      message: 'Internal server error during authentication.'
    });
  }
};

/**
 * Higher-order function to wrap API handlers with authentication
 * This returns a new handler that first authenticates the user before calling the original handler
 */
export const withAuth = (handler: NextApiHandler): NextApiHandler => {
  return async (req: AuthenticatedRequest, res: NextApiResponse) => {
    // First run the authentication middleware
    await new Promise<void>((resolve, reject) => {
      authenticateUser(req, res, resolve);
    });

    // If user was not authenticated, the response would have been sent by authenticateUser
    // So we only proceed if the response hasn't been sent yet
    if (!res.writableEnded && req.user) {
      return handler(req, res);
    }
    // If response was already sent (e.g., 401 error), don't call the handler
  };
};

/**
 * Middleware to ensure that only authenticated users can access certain routes
 */
export const requireAuth = (handler: NextApiHandler): NextApiHandler => {
  return async (req: AuthenticatedRequest, res: NextApiResponse) => {
    // If there's no user attached to the request (meaning authentication failed)
    if (!req.user) {
      // The authenticateUser middleware should have already sent the error response
      // But if for some reason we get here without a user and response not sent
      if (res.writableEnded) return; // Response already sent

      res.status(401).json({
        success: false,
        message: 'Authentication required to access this resource'
      });
      return;
    }

    // If user is authenticated, proceed to the original handler
    return handler(req, res);
  };
};

/**
 * Role-based access control middleware
 * This can be extended to check user roles/permissions
 */
export const requireRole = (roles: string[]) => {
  return (handler: NextApiHandler): NextApiHandler => {
    return async (req: AuthenticatedRequest, res: NextApiResponse) => {
      // First ensure user is authenticated
      if (!req.user || !req.user.authenticated) {
        if (res.writableEnded) return; // Response already sent

        res.status(401).json({
          success: false,
          message: 'Authentication required to access this resource'
        });
        return;
      }

      // Check if user has one of the required roles
      // For now, we'll assume all authenticated users have the 'user' role
      // This would need to be extended based on the actual user role system
      const userHasRequiredRole = roles.includes('user') || roles.includes('admin');

      if (!userHasRequiredRole) {
        res.status(403).json({
          success: false,
          message: 'Insufficient permissions to access this resource'
        });
        return;
      }

      // If user has required role, proceed to the original handler
      return handler(req, res);
    };
  };
};

/**
 * Middleware to attach user profile information to the request
 */
export const attachUserProfile = async (req: AuthenticatedRequest, res: NextApiResponse, next: () => void) => {
  if (!req.user || !req.user.id) {
    // If user is not authenticated, just call next without attaching profile
    next();
    return;
  }

  try {
    // Fetch user profile from database
    const userProfile = await UserModel.findById(req.user.id);

    if (!userProfile) {
      res.status(401).json({
        success: false,
        message: 'User profile not found'
      });
      return;
    }

    // Attach the user profile to the request for subsequent handlers
    req.user.profile = userProfile;

    next();
  } catch (error) {
    console.error('Error attaching user profile:', error);
    res.status(500).json({
      success: false,
      message: 'Internal server error while retrieving user profile'
    });
  }
};