# Implementation Plan: Docusaurus Authentication System

**Branch**: `005-docusaurus-auth` | **Date**: 2025-12-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-docusaurus-auth/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement authentication system for the Physical AI & Humanoid Robotics textbook built with Docusaurus. The system will use Better Auth for authentication, Next.js API routes for backend services, and Neon Postgres for database storage. Core features include user registration with onboarding questions, login/logout with persistent sessions, password reset, and GDPR-compliant account deletion. The system must integrate seamlessly with existing Docusaurus deployment without breaking changes.

## Technical Context

**Language/Version**: TypeScript/JavaScript (for Next.js API routes), Node.js 18+
**Primary Dependencies**: Better Auth (v1.0+), Next.js (v14+), Neon Postgres driver, Docusaurus v3
**Storage**: Neon Serverless Postgres database with user, session, and password reset token tables
**Email Service**: Resend (https://resend.com/) - Free tier includes 100 emails/day (recommended), with alternatives: SendGrid or NodeMailer
**Testing**: Jest for unit/integration tests, Playwright for end-to-end tests
**Target Platform**: Web application (Docusaurus frontend + Next.js API backend)
**Project Type**: Web application (frontend Docusaurus integration + backend Next.js API)
**Performance Goals**: Registration completes in 6-8 seconds, password reset email delivered in 7-10 seconds
**Constraints**: Support 1000 concurrent users, maintain 99.5% uptime, GDPR compliance for data deletion
**Scale/Scope**: 1000 concurrent users, mobile-responsive (320px+ screens), dark/light mode support

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Feature Requirements Alignment

✅ **Authentication System**: Plan implements Better-Auth with secure session management per constitution requirements

✅ **User Profiling**: System collects software/hardware background information during registration

✅ **Backend Requirements**: Uses Neon Serverless Postgres as specified in constitution

✅ **Frontend Requirements**: Integrates with Docusaurus v3 as specified in constitution

✅ **Performance Benchmarks**: Plan accounts for performance goals (registration in 6-8 seconds)

✅ **UI/UX Standards**: Supports both dark/light modes as per constitution requirements

**Status**: All constitution requirements satisfied - No violations identified

## Phase 1 Completion Summary

**Phase 0 (Research)**: ✅ COMPLETED
- research.md created with technology research and decision log
- All architecture decisions documented with rationale

**Phase 1 (Design & Contracts)**: ✅ COMPLETED
- data-model.md created with complete entity definitions
- contracts/ directory created with API specifications
- quickstart.md created for developer onboarding
- Agent context updated with new technology stack

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-auth/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Physical-AI-Humanoid-Robotics-Textbook/ (existing repo)
├── docs/                             # Existing Docusaurus content - NO CHANGES
├── src/
│   ├── components/
│   │   ├── auth/                    # NEW - auth UI components
│   │   │   ├── RegisterForm.tsx
│   │   │   ├── LoginForm.tsx
│   │   │   ├── ResetPasswordForm.tsx
│   │   │   ├── AccountDeletionForm.tsx
│   │   │   └── OnboardingQuestions.tsx
│   │   └── ...                      # Existing components remain unchanged
│   └── pages/                       # Existing Docusaurus pages
├── docusaurus.config.js             # Existing - minor updates for auth integration
├── sidebars.js                      # Existing - may need updates to show auth-required content
├── auth-backend/                    # NEW - Next.js API for Better Auth
│   ├── pages/
│   │   └── api/
│   │       ├── auth/
│   │       │   ├── register.ts
│   │       │   ├── login.ts
│   │       │   ├── logout.ts
│   │       │   ├── reset-password.ts
│   │       │   └── delete-account.ts
│   │       └── users/
│   │           ├── profile.ts
│   │           └── onboarding.ts
│   ├── middleware/
│   │   └── auth.ts
│   ├── lib/
│   │   ├── auth.ts
│   │   ├── database.ts
│   │   ├── email.ts
│   │   └── validation.ts
│   ├── types/
│   │   └── auth.ts
│   ├── next.config.js
│   ├── package.json
│   └── tsconfig.json
├── migrations/                      # Database migration scripts
│   ├── 001_create_users_table.sql
│   ├── 002_create_password_reset_tokens_table.sql
│   └── 003_add_user_profile_fields.sql
└── package.json                     # Existing Docusaurus package.json
```

**Structure Decision**: The existing Docusaurus site remains unchanged at repository root while a new auth-backend/ folder is created for the Next.js API. This allows for proper separation of authentication concerns while maintaining the existing Docusaurus site structure.

## Deployment Architecture

### Frontend: Docusaurus Site
- **Platform**: GitHub Pages (existing workflow continues unchanged)
- **URL**: https://physical-ai-humanoid-robotics-textbook.github.io/
- **Build Process**: Standard Docusaurus build process (`npm run build`)
- **Workflow**: GitHub Actions will continue to deploy updates to GitHub Pages

### Backend: Next.js API
- **Platform**: Vercel (free tier, seamless Next.js support)
- **URL**: https://docusaurus-auth-backend.vercel.app/ (example)
- **Build Process**: Standard Next.js deployment to Vercel
- **Environment**: Separate deployment from Docusaurus frontend

### CORS Configuration
- **Frontend Origin**: GitHub Pages URL (e.g., https://physical-ai-humanoid-robotics-textbook.github.io)
- **Required Headers**:
  - Access-Control-Allow-Origin: "https://physical-ai-humanoid-robotics-textbook.github.io" (or configured domain)
  - Access-Control-Allow-Credentials: true
  - Access-Control-Allow-Methods: "GET, POST, PUT, DELETE, OPTIONS"
  - Access-Control-Allow-Headers: "Content-Type, Authorization, X-Requested-With"

### Environment Variables

#### Docusaurus (docusaurus.config.js/.env)
```env
NEXT_PUBLIC_API_BASE_URL="https://docusaurus-auth-backend.vercel.app"
NEXT_PUBLIC_APP_NAME="Physical AI & Humanoid Robotics Textbook"
NEXT_PUBLIC_SITE_URL="https://physical-ai-humanoid-robotics-textbook.github.io"
```

#### Next.js API (auth-backend/.env.local)
```env
DATABASE_URL="postgresql://username:password@ep-xxxx.us-east-1.aws.neon.tech/auth_db"
BETTER_AUTH_SECRET="your-super-secret-jwt-secret"
BETTER_AUTH_URL="https://docusaurus-auth-backend.vercel.app"
NEXTAUTH_URL="https://docusaurus-auth-backend.vercel.app"
RESEND_API_KEY="your-resend-api-key"
SENDER_EMAIL="noreply@yourdomain.com"
CORS_ORIGIN="https://physical-ai-humanoid-robotics-textbook.github.io"
NODE_ENV="production"  # or "development"
```

**Deployment Decision**: Decoupled deployment strategy allows independent scaling and maintenance of frontend and backend.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (None) | | |
