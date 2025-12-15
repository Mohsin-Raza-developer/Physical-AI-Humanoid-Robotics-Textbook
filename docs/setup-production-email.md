# Production Email Setup Guide (Resend)

To make "Forgot Password" and other email features work in **Production**, you must configure a real email service. We use **Resend** because it is reliable and developer-friendly.

## Step 1: Get API Key
1. Go to [Resend.com](https://resend.com).
2. Sign up or Log in.
3. Click "API Keys" in the sidebar.
4. Click "Create API Key".
5. Give it a name (e.g., "Production Key") and Permission "Full Access" or "Sending Access".
6. **Copy the Key** (starts with `re_...`).

## Step 2: Configure Backend
Open your `auth-backend/.env` (or `.env.local`) file and add/update:

```env
RESEND_API_KEY=re_123456789...  <-- Paste your key here
SENDER_EMAIL=onboarding@resend.dev
```

*Note: For testing, you can use `onboarding@resend.dev`. For real production, you must verify your own domain on Resend and use `noreply@yourdomain.com`.*

## Step 3: Verify Domain (Important for Production)
For Test mode, you can only send emails to your *own* email address (the one you signed up with on Resend).
To send to **ANY** user in production:
1. Go to "Domains" on Resend.
2. Add your domain (e.g., `robotics-textbook.com`).
3. Add the DNS records (DKIM/SPF) to your DNS provider (GoDaddy, Namecheap, Vercel, etc.).
4. Once verified, change `SENDER_EMAIL` in `.env` to `noreply@yourdomain.com`.

## Troubleshooting
- If you see `API key is invalid`, you copied it wrong.
- If email is not received, check Spam folder.
- In `npm run dev`, you can see the Reset Link in the terminal as a backup.
