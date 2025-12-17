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

  const sql = neon(process.env.DATABASE_URL);

  // Wrap neon in a compatibility layer to match pg.Pool interface
  db = {
    query: async (text: string, params?: any[]) => {
      // Neon driver returns the rows directly as an array
      const rows = await sql(text, params || []);
      // Return object structure matching pg.Pool result
      return {
        rows,
        rowCount: rows.length,
        command: 'UNKNOWN',
        oid: 0,
        fields: []
      };
    },
    connect: async () => {
      // No-op for serverless HTTP
      return { release: () => { } };
    }
  };
} else {
  // Use regular PostgreSQL Pool for development
  if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL is required for database connection');
  }

  // DEBUG: Check which DB we are connecting to
  console.log('[Database] Connecting to:', process.env.DATABASE_URL?.split('@')[1] || 'Unknown');
  console.log('[Database] SSL Mode:', process.env.NODE_ENV === 'production' ? 'Production' : 'Development (Forced SSL)');

  const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
    // FORCE SSL for Neon (even in development)
    ssl: true,
  });

  db = {
    query: async (text: string, params?: any[]) => {
      const start = Date.now();
      try {
        const res = await pool.query(text, params);
        // console.log(`[Database] Query took ${Date.now() - start}ms`, { text });
        return res;
      } catch (e) {
        console.error('[Database] Query Failed:', text, e);
        throw e;
      }
    },
    connect: () => pool.connect(),
  };
}

export default db;