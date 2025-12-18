import { RateLimiterMemory } from 'rate-limiter-flexible';
import type { NextApiRequest, NextApiResponse } from 'next';

// Email-based rate limiter - tracks attempts per email address
const emailAuthLimiter = new RateLimiterMemory({
  points: 7, // Maximum 7 attempts per email
  duration: 60 * 60, // Per 1 hour (in seconds)
});

// Rate limiting for auth endpoints (email-based)
export const authRateLimit = async (
  req: NextApiRequest,
  res: NextApiResponse,
  email?: string
): Promise<boolean> => {
  // If email is not provided, allow the request (no rate limiting)
  if (!email) {
    return true;
  }

  // Normalize email to lowercase for consistent tracking
  const normalizedEmail = email.toLowerCase().trim();

  try {
    // Consume a point for this email address
    await emailAuthLimiter.consume(normalizedEmail);
    return true; // Rate limit not exceeded
  } catch (rejRes) {
    // If rate limit is exceeded for this email
    res.status(429).json({
      success: false,
      message: 'Too many authentication attempts with this email, please try again later.',
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