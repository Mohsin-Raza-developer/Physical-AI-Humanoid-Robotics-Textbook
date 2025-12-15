import { NextApiHandler, NextApiRequest, NextApiResponse } from 'next';

// CORS configuration based on the deployment architecture specified in plan.md
const allowedOrigins = [
  process.env.CORS_ORIGIN || 'http://localhost:3000', // Default to localhost for development
  'https://physical-ai-humanoid-robotics-textbook.github.io', // Production GitHub Pages
];

// Check if we're in production or development to set appropriate origins
if (process.env.NODE_ENV === 'production') {
  allowedOrigins.push('https://physical-ai-humanoid-robotics-textbook.github.io');
} else {
  // In development, allow common local development ports
  allowedOrigins.push('http://localhost:3000');
  allowedOrigins.push('http://localhost:3001');
  allowedOrigins.push('http://localhost:3002');
  allowedOrigins.push('http://localhost:8080');
  allowedOrigins.push('http://localhost:8000');
}

// CORS middleware function
export const cors = (req: NextApiRequest, res: NextApiResponse) => {
  return new Promise<void>((resolve) => {
    // Get the origin of the request
    const origin = req.headers.origin;

    // Check if the origin is allowed
    if (origin && allowedOrigins.includes(origin)) {
      res.setHeader('Access-Control-Allow-Origin', origin);
    } else {
      // If the origin isn't explicitly allowed, use the default from environment or localhost
      res.setHeader('Access-Control-Allow-Origin', allowedOrigins[0]);
    }

    // Set other CORS headers as specified in deployment architecture
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization, Content-Length, Cache-Control');
    res.setHeader('Access-Control-Allow-Credentials', 'true'); // Allow credentials to be sent

    // Handle preflight requests
    if (req.method === 'OPTIONS') {
      res.status(200).end();
      resolve();
      return;
    }

    resolve();
  });
};

// Higher-order function to wrap API handlers with CORS
export const withCors = (handler: NextApiHandler): NextApiHandler => {
  return async (req: NextApiRequest, res: NextApiResponse) => {
    await cors(req, res);

    // Call the original handler
    return handler(req, res);
  };
};