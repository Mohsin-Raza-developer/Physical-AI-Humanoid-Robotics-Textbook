"""
FastAPI application entry point.

Initializes the ChatKit Gemini backend with CORS, routers, and middleware.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.routers import threads, messages
import sentry_sdk

# Initialize Sentry (optional)
if settings.SENTRY_DSN:
    sentry_sdk.init(dsn=settings.SENTRY_DSN, traces_sample_rate=1.0)

# Create FastAPI app
app = FastAPI(
    title="ChatKit Gemini Backend",
    description="AI chatbot backend for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS middleware configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type"],
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"]
)

# Include routers
app.include_router(threads.router)
app.include_router(messages.router)


# Health check endpoint
@app.get("/health", tags=["Health"])
async def health_check():
    """
    Health check endpoint for monitoring and deployment verification.

    Returns:
        dict: Status information
    """
    return {
        "status": "healthy",
        "environment": settings.ENVIRONMENT
    }


# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize services on application startup."""
    pass


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup resources on application shutdown."""
    from app.database import close_db
    await close_db()
