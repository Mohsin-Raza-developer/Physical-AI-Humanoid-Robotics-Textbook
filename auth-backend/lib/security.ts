import type { NextApiRequest, NextApiResponse } from 'next';
import helmet from 'helmet';

// Apply security headers to API responses
export const applySecurityHeaders = (req: NextApiRequest, res: NextApiResponse) => {
  // Use helmet to set various security headers
  helmet({
    contentSecurityPolicy: {
      directives: {
        defaultSrc: ["'self'"],
        styleSrc: ["'self'", "'unsafe-inline'", 'fonts.googleapis.com'],
        fontSrc: ["'self'", 'fonts.gstatic.com'],
        imgSrc: ["'self'", 'data:', 'https:'],
        scriptSrc: ["'self'"],
        connectSrc: ["'self'", process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:3001'],
      },
    },
    hsts: {
      maxAge: 63072000, // 2 years
      includeSubDomains: true,
      preload: true,
    },
    frameguard: {
      action: 'deny',
    },
    referrerPolicy: {
      policy: 'strict-origin-when-cross-origin',
    },
  })(req, res, () => {
    // Continue to the next middleware
  });
};