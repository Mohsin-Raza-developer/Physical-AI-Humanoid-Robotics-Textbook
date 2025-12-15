import { betterAuth } from 'better-auth';

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

  // Database adapter for Neon Postgres
  database: {
    url: process.env.DATABASE_URL || "",
  },

  // Additional user fields specific to our application
  user: {
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

// Export helper functions for session management
export const {
  signIn,
  signUp,
  signOut,
  getSession,
  getAccount,
  updateSession,
  deleteSession
} = auth;