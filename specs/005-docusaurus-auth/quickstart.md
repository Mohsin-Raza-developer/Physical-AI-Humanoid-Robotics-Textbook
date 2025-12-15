# Quickstart Guide: Docusaurus Authentication System

**Feature**: 005-docusaurus-auth  
**Date**: 2025-12-13  
**Status**: Complete

## Overview

This guide provides a quick start for developers to set up and run the Docusaurus Authentication System locally. It covers environment setup, database configuration, and initial development steps.

## Prerequisites

Before starting, ensure you have:

- Node.js (v18 or higher)
- npm or yarn package manager
- Docker (for local database setup) or access to Neon Postgres
- Git

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
# Install auth-backend dependencies
cd auth-backend
npm install

# Install Docusaurus dependencies at repository root
cd ..  # back to repository root where Docusaurus is located
npm install
```

### 3. Environment Configuration

Create a `.env.local` file in the auth-backend directory with the following variables:

```env
# Database Configuration
DATABASE_URL="postgresql://username:password@localhost:5432/auth_db"

# Better Auth Configuration
BETTER_AUTH_SECRET="your-super-secret-jwt-secret"
BETTER_AUTH_URL="http://localhost:3000"

# Email Service Configuration (for password reset)
RESEND_API_KEY="your-resend-api-key"
SENDER_EMAIL="noreply@yourdomain.com"

# Next.js Configuration
NEXT_PUBLIC_APP_URL="http://localhost:3000"
CORS_ORIGIN="http://localhost:3000"
```

## Database Setup

### 1. Set up Neon Postgres Database

- Create a Neon account and project
- Copy the connection string and update `DATABASE_URL` in your `.env` file

### 2. Run Database Migrations

```bash
# Using Prisma (if chosen as ORM)
npx prisma migrate dev

# Or run SQL migration files directly
psql -d your_database_url -f migrations/001_create_users_table.sql
psql -d your_database_url -f migrations/002_create_password_reset_tokens_table.sql
psql -d your_database_url -f migrations/003_add_user_profile_fields.sql
```

## Running the Application

### 1. Start the Auth Backend

```bash
cd auth-backend
npm run dev
```

The auth backend will start on `http://localhost:3001` (or your configured port).

### 2. Start the Docusaurus Site

```bash
cd ..  # to repository root where Docusaurus is located
npm run start
```

The Docusaurus site will start on `http://localhost:3000`.

## Key Development Commands

### Auth Backend Commands

```bash
# Run development server
cd auth-backend
npm run dev

# Run tests
npm test

# Build for production
npm run build

# Run database migrations
npx prisma migrate deploy  # if using Prisma
```

### Docusaurus Commands

```bash
# Run development server (from repository root)
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

## API Testing

### Test User Registration

```bash
curl -X POST http://localhost:3001/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123!",
    "softwareLevel": "Intermediate",
    "hardwareAccess": "RTX GPU"
  }'
```

### Test User Login

```bash
curl -X POST http://localhost:3001/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123!"
  }'
```

## Integration with Docusaurus

### 1. Adding Authentication Components

The authentication components can be imported into Docusaurus pages:

```jsx
// In a Docusaurus MDX file
import { LoginForm, RegisterForm } from '@site/src/components/auth';

<LoginForm />
// or
<RegisterForm />
```

### 2. Protecting Content

Use the ProtectedRoute component to protect specific documentation sections:

```jsx
// In a Docusaurus page
import ProtectedRoute from '@site/src/components/common/ProtectedRoute';

export default function ProtectedPage() {
  return (
    <ProtectedRoute>
      <div>
        {/* Only authenticated users can see this content */}
        <h1>Protected Content</h1>
        <p>This content is only accessible to logged-in users.</p>
      </div>
    </ProtectedRoute>
  );
}
```

## Testing the Authentication Flow

1. **Register a new user** using the registration form or API
2. **Check email** for verification (if implemented)
3. **Log in** with the registered credentials
4. **Access protected content** to verify authentication works
5. **Test password reset** flow
6. **Test account deletion** (in development environment)

## Common Issues and Solutions

### Database Connection Issues
- Verify your Neon Postgres connection URL is correct
- Check that your database allows connections from your IP
- Ensure your `.env` file is properly loaded

### CORS Issues
- Make sure your backend allows requests from your frontend origin
- Check backend CORS configuration in middleware

### Email Delivery Issues
- Verify Resend API key in your environment
- Check that your domain is verified with Resend
- For development, verify your API key and test email sending

## Next Steps

1. Complete the full API implementation based on contracts
2. Implement UI components for authentication flows
3. Add comprehensive tests for all authentication functions
4. Review and enhance security measures
5. Performance test with 1000 simulated concurrent users