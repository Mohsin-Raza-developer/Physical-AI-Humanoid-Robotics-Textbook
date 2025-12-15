# 🚀 Vercel Deployment Guide - Auth Backend

Complete guide to deploy the Authentication Backend to Vercel.

---

## 📋 Prerequisites / ضروری چیزیں

Before starting, make sure you have:
- ✅ GitHub Account
- ✅ Vercel Account ([vercel.com](https://vercel.com))
- ✅ Neon PostgreSQL Database ([neon.tech](https://neon.tech))
- ✅ Gmail App Password (for email verification)

---

## 🎯 Deployment Strategy / حکمت عملی

We have **2 options**:

### Option A: Separate Repository (✅ Recommended)
**Faida (Advantage):** Clean deployment, easier management  
**Steps:**
1. Create new GitHub repo: `auth-backend`
2. Copy `auth-backend/` folder to new repo
3. Deploy from Vercel

### Option B: Monorepo (Current Setup)
**Faida (Advantage):** Keep everything together  
**Steps:**
1. Deploy directly from current repo
2. Set `Root Directory: auth-backend` in Vercel

---

## 📦 Step 1: Prepare Backend for Deployment

### 1.1 Create `vercel.json` Configuration

In `auth-backend/` folder, create `vercel.json`:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/next"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "/api/$1"
    }
  ],
  "env": {
    "NODE_ENV": "production"
  }
}
```

### 1.2 Update `package.json` Scripts

Ensure these scripts exist in `auth-backend/package.json`:

```json
{
  "scripts": {
    "dev": "next dev -p 3002",
    "build": "next build",
    "start": "next start -p 3002",
    "vercel-build": "next build"
  }
}
```

---

## 🌐 Step 2: Setup Vercel Project

### 2.1 Import Project
1. Go to [vercel.com/dashboard](https://vercel.com/dashboard)
2. Click **"Add New Project"**
3. Import your GitHub repository
4. Select the repository containing auth-backend

### 2.2 Configure Build Settings

**Framework Preset:** `Next.js`

**Root Directory:** (if monorepo)
```
auth-backend
```

**Build Command:** (leave default or use)
```bash
npm run build
```

**Output Directory:**
```
.next
```

**Install Command:**
```bash
npm install
```

---

## 🔐 Step 3: Environment Variables Setup

In Vercel Project Settings → Environment Variables, add **ALL** of these:

### Database Configuration
```env
DATABASE_URL=postgresql://YOUR_NEON_USER:YOUR_PASSWORD@YOUR_NEON_HOST/neondb?sslmode=require
```
**کہاں سے لیں:** [Neon Dashboard](https://console.neon.tech) → Connection String

---

### JWT & Security
```env
JWT_SECRET=your-super-secret-jwt-key-min-32-characters-long
```
**کیسے بنائیں:**
```bash
# PowerShell (Windows)
-join ((65..90) + (97..122) + (48..57) | Get-Random -Count 32 | % {[char]$_})

# Or use this site: https://randomkeygen.com/
```

---

### Email Configuration (Gmail)
```env
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASS=your-16-digit-app-password
EMAIL_FROM=noreply@yourdomain.com
```

**📧 Gmail App Password کیسے بنائیں:**
1. Go to [Google Account Security](https://myaccount.google.com/security)
2. Enable **2-Step Verification**
3. Go to **App Passwords**
4. Create password for "Mail"
5. Copy the 16-character password

---

### Site URL & CORS
```env
NEXT_PUBLIC_SITE_URL=https://your-auth-backend.vercel.app
COOKIE_DOMAIN=.vercel.app
ALLOWED_ORIGINS=https://your-frontend-site.github.io,https://localhost:3000
```

**یاد رکھیں:** Replace URLs with your actual deployed URLs after deployment.

---

### Node Environment
```env
NODE_ENV=production
```

---

## 🗄️ Step 4: Database Migration

After deployment, run migrations on Vercel:

### Option A: Using Vercel CLI
```bash
# Install Vercel CLI
npm i -g vercel

# Link project
vercel link

# Run migration command
vercel env pull .env.production
# Then manually run migrations using a script
```

### Option B: Use Neon SQL Editor
1. Go to [Neon Console](https://console.neon.tech)
2. Open SQL Editor
3. Copy contents from `migrations/001_create_users.sql`
4. Execute the SQL

---

## 🚀 Step 5: Deploy

### From Vercel Dashboard
1. Click **"Deploy"**
2. Wait for build to complete (2-3 minutes)
3. Copy the deployment URL: `https://your-project.vercel.app`

### From CLI (Alternative)
```bash
cd auth-backend
vercel --prod
```

---

## 🔗 Step 6: Update Frontend Configuration

After successful deployment, update the root `.env`:

```env
# Production
NEXT_PUBLIC_API_BASE_URL=https://your-auth-backend.vercel.app

# Development (keep for local testing)
# NEXT_PUBLIC_API_BASE_URL=http://localhost:3002
```

**GitHub Pages deployment کے لیے:**
Create `.env.production` in root:
```env
NEXT_PUBLIC_API_BASE_URL=https://your-auth-backend.vercel.app
```

---

## ✅ Step 7: Testing Deployment

### 7.1 Health Check
Visit: `https://your-auth-backend.vercel.app/api/health`

**Expected Response:**
```json
{
  "status": "ok",
  "timestamp": "2025-12-16T..."
}
```

### 7.2 Test Registration
Using Postman or curl:
```bash
curl -X POST https://your-auth-backend.vercel.app/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{
    "firstName": "Test",
    "lastName": "User",
    "email": "test@example.com",
    "password": "Test123!",
    "softwareLevel": "Beginner",
    "hardwareAccess": "Laptop/Cloud"
  }'
```

---

## 🐛 Troubleshooting / مسائل کا حل

### Issue 1: "Module not found" Error
**حل:**
- Check `package.json` dependencies
- Run `npm install` locally
- Redeploy

### Issue 2: Database Connection Failed
**چیک کریں:**
- ✅ `DATABASE_URL` correct hai?
- ✅ Neon database running hai?
- ✅ SSL mode enabled hai (`?sslmode=require`)?

### Issue 3: SMTP Authentication Failed
**چیک کریں:**
- ✅ Gmail App Password correct hai (NOT your Google password)?
- ✅ 2-Step Verification enabled hai?
- ✅ SMTP settings correct hain?

### Issue 4: CORS Errors
**حل:**
Update `ALLOWED_ORIGINS`:
```env
ALLOWED_ORIGINS=https://mohsin-raza-developer.github.io,https://your-custom-domain.com
```

### Issue 5: 500 Internal Server Error
**Logs دیکھیں:**
1. Vercel Dashboard → Your Project
2. Click on failing deployment
3. View **Function Logs**
4. Check error details

---

## 📊 Post-Deployment Checklist

- [ ] Health endpoint working: `/api/health`
- [ ] Registration working: `/api/auth/register`
- [ ] Login working: `/api/auth/login`
- [ ] Email verification sending
- [ ] Frontend can connect to backend
- [ ] CORS configured properly
- [ ] Environment variables set correctly
- [ ] Database migrations applied
- [ ] Error logs clean (no critical errors)

---

## 🔄 Updating Deployment

### Method 1: Auto-Deploy (Recommended)
1. Push code to GitHub
2. Vercel automatically deploys

### Method 2: Manual Deploy
```bash
cd auth-backend
vercel --prod
```

### Rolling Back
1. Vercel Dashboard → Deployments
2. Find previous working deployment
3. Click "Promote to Production"

---

## 🌍 Custom Domain (Optional)

### Add Custom Domain
1. Vercel Dashboard → Settings → Domains
2. Add domain: `auth.yourdomain.com`
3. Update DNS records (Vercel will show instructions)
4. Update environment variables:
```env
NEXT_PUBLIC_SITE_URL=https://auth.yourdomain.com
COOKIE_DOMAIN=.yourdomain.com
```

---

## 📞 Support & Resources

- **Vercel Docs:** [vercel.com/docs](https://vercel.com/docs)
- **Neon Docs:** [neon.tech/docs](https://neon.tech/docs)
- **Next.js Deployment:** [nextjs.org/docs/deployment](https://nextjs.org/docs/deployment)

---

## 🎉 Deployment Complete!

Aapka backend ab live hai! 🚀

**Next Steps:**
1. Test all API endpoints
2. Deploy frontend to GitHub Pages
3. Update frontend environment variables
4. Monitor logs for errors
5. Setup monitoring (Vercel Analytics)

---

**کوئی مسئلہ ہو تو بتائیں! Happy Deploying! 🎊**
