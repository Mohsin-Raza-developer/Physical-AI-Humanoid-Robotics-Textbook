import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

// Better Auth configuration with our custom requirements
export const auth = betterAuth({
  secret: process.env.BETTER_AUTH_SECRET || 'fallback-secret-for-development',
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3001',
  // Session configuration with 30-day timeout
  session: {
    expiresIn: 30 * 24 * 60 * 60, // 30 days in seconds (default is 7 days)
    rememberMe: true, // Enable persistent sessions until manual logout
  },
  // Email/password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // May be changed later as needed
  },

  // Database adapter using pg Pool (works for both local and Neon)
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: true, // Neon requires SSL
  }),

  // Additional user fields specific to our application
  user: {
    modelName: "users", // Tell Better Auth to use 'users' table instead of 'user'
    additionalFields: {
      software_level: {
        type: 'string',
        required: true,
      },
      hardware_access: {
        type: 'string',
        required: true,
      },
    },
  },
});

// Export the authentication instance
export default auth;