import { NextApiHandler, NextApiRequest, NextApiResponse } from 'next';

// CORS configuration based on the deployment architecture
const allowedOrigins = [
  // Production URLs
  'https://physical-ai-humanoid-robotics-textbook.github.io',
  'https://mohsin-raza-developer.github.io',

  // Local Development URLs
  'http://localhost:3000',
  'http://localhost:3001',
  'http://localhost:3002',
  'http://localhost:8080',
  'http://localhost:8000',

  // Environment variable override
  ...(process.env.CORS_ORIGIN ? [process.env.CORS_ORIGIN] : [])
];

// Deduplicate the array just in case
const uniqueAllowedOrigins = allowedOrigins.filter((value, index, self) =>
  self.indexOf(value) === index
);

// CORS middleware function
export const cors = (req: NextApiRequest, res: NextApiResponse) => {
  return new Promise<void>((resolve) => {
    // Get the origin of the request
    const origin = req.headers.origin;

    // Check if the origin is allowed
    if (origin && uniqueAllowedOrigins.includes(origin)) {
      res.setHeader('Access-Control-Allow-Origin', origin);
    } else {
      // If the origin isn't explicitly allowed, use the default from environment or localhost
      res.setHeader('Access-Control-Allow-Origin', uniqueAllowedOrigins[0]);
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

    // If headers/response sent (e.g. OPTIONS), stop here
    if (res.writableEnded || res.headersSent) {
      return;
    }

    // Call the original handler
    return handler(req, res);
  };
};