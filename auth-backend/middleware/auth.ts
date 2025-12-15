import { NextApiHandler, NextApiRequest, NextApiResponse } from 'next';
import { User } from '../lib/user';

export interface AuthenticatedRequest extends NextApiRequest {
  user?: any; // User object attached by the middleware
}

/**
 * Middleware to verify authentication for API endpoints
 * This middleware checks if the user is authenticated and attaches user info to the request
 */
export const authenticateUser = async (req: AuthenticatedRequest, res: NextApiResponse, next: () => void) => {
  try {
    // Extract the authentication token from the request
    // Usually this comes from the Authorization header or cookies
    const authHeader = req.headers.authorization;
    const cookieHeader = req.headers.cookie;
    
    // For Better Auth, we typically look for session cookies or authorization headers
    let sessionToken = null;
    
    // Check for Authorization header
    if (authHeader && authHeader.startsWith('Bearer ')) {
      sessionToken = authHeader.substring(7, authHeader.length); // Remove 'Bearer ' prefix
    } 
    // Check for session cookie (common in Better Auth)
    else if (cookieHeader) {
      const cookies = cookieHeader.split(';');
      const sessionCookie = cookies.find(c => c.trim().startsWith('better-auth.session-token='));
      if (sessionCookie) {
        sessionToken = sessionCookie.split('=')[1];
      }
    }

    // If no token provided
    if (!sessionToken) {
      res.status(401).json({ 
        success: false, 
        message: 'Access denied. No authentication token provided.' 
      });
      return;
    }

    // For now, we'll implement a simplified version. 
    // In a real implementation with Better Auth, we'd call the session validation
    // Since we don't have Better Auth fully configured yet, we'll skip actual validation
    // and just check that a token exists (in real implementation, we'd validate the token)
    
    // For demonstration, we'll just attach a placeholder user
    // In reality, we would validate the session token with Better Auth
    // const session = await validateSession(sessionToken);
    
    // Placeholder - in real implementation, we would validate the token
    // and extract the user information from the validated session
    req.user = { id: 'demo-user-id', authenticated: true };

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
    const userProfile = await User.findById(req.user.id);
    
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