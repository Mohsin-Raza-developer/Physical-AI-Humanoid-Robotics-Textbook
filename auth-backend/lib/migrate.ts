import { Pool } from 'pg';
import fs from 'fs';
import path from 'path';

const db = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : undefined,
});

async function runMigrations() {
  try {
    console.log('Starting database migrations...');
    
    // Read all SQL migration files from the migrations directory
    const migrationsDir = path.join(__dirname, '../migrations');
    const migrationFiles = fs.readdirSync(migrationsDir)
      .filter(file => file.endsWith('.sql'))
      .sort(); // Sort to ensure migrations run in order
    
    for (const fileName of migrationFiles) {
      const filePath = path.join(migrationsDir, fileName);
      const sql = fs.readFileSync(filePath, 'utf8');
      
      console.log(`Running migration: ${fileName}`);
      
      // Execute the migration
      await db.query(sql);
      
      console.log(`Completed migration: ${fileName}`);
    }
    
    console.log('All migrations completed successfully!');
  } catch (error) {
    console.error('Migration failed:', error);
    process.exit(1); // Exit with error code
  } finally {
    await db.end(); // Close the database connection
  }
}

// Run migrations if this file is executed directly
if (require.main === module) {
  runMigrations();
}

export default runMigrations;