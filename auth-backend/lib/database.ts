import { Pool } from 'pg';
import { neon } from '@neondatabase/serverless';

// Check if we're in a serverless environment (Vercel)
const isServerless = process.env.VERCEL === '1' || process.env.NODE_ENV === 'production';

let db: Pool | any;

if (isServerless) {
  // Use Neon's serverless driver for Vercel deployments
  if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL is required for Neon database connection');
  }

  db = neon(process.env.DATABASE_URL);
} else {
  // Use regular PostgreSQL Pool for development
  if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL is required for database connection');
  }

  const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : undefined,
  });

  db = {
    query: (text: string, params?: any[]) => pool.query(text, params),
    connect: () => pool.connect(),
  };
}

export default db;