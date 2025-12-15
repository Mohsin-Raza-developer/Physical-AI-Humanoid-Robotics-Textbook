import { RateLimiterMemory } from 'rate-limiter-flexible';
import type { NextApiRequest, NextApiResponse } from 'next';

// Create rate limiter configuration
const authLimiter = new RateLimiterMemory({
  // Limit to 20 requests per 15 minutes per IP during development
  // In production, this should be set to a lower value like 5 requests per 15 minutes
  points: process.env.NODE_ENV === 'production' ? 5 : 20, // Number of points (requests)
  duration: 15 * 60, // Per 15 minutes (in seconds)
});

// Specific rate limiter for auth endpoints
export const authRateLimit = async (req: NextApiRequest, res: NextApiResponse): Promise<boolean> => {
  // Get IP address from request
  const ip = req.headers['x-forwarded-for']?.toString().split(',')[0].trim() ||
             req.connection.remoteAddress ||
             req.socket.remoteAddress ||
             (req as any).ip ||
             '127.0.0.1';

  try {
    // Consume a point for this IP
    await authLimiter.consume(ip);
    return true; // Rate limit not exceeded
  } catch (rejRes) {
    // If rate limit is exceeded
    res.status(429).json({
      success: false,
      message: 'Too many authentication attempts, please try again later.',
    });
    return false;
  }
};

// General rate limiter (for non-auth endpoints)
const generalLimiter = new RateLimiterMemory({
  points: process.env.NODE_ENV === 'production' ? 100 : 200, // 100/200 requests
  duration: 15 * 60, // Per 15 minutes
});

export const generalRateLimit = async (req: NextApiRequest, res: NextApiResponse): Promise<boolean> => {
  const ip = req.headers['x-forwarded-for']?.toString().split(',')[0].trim() ||
             req.connection.remoteAddress ||
             req.socket.remoteAddress ||
             (req as any).ip ||
             '127.0.0.1';

  try {
    await generalLimiter.consume(ip);
    return true;
  } catch (rejRes) {
    res.status(429).json({
      success: false,
      message: 'Too many requests, please try again later.',
    });
    return false;
  }
};