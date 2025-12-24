"""
Application configuration using Pydantic settings.

Loads configuration from environment variables with validation.
"""
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, validator


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Database
    DATABASE_URL: str = Field(
        ...,
        description="PostgreSQL connection string (async format: postgresql+asyncpg://...)"
    )

    # AI Services
    GEMINI_API_KEY: str = Field(..., description="Google Gemini API key")
    COHERE_API_KEY: str = Field(..., description="Cohere API key for embeddings")

    # Vector Database
    QDRANT_URL: str = Field(..., description="Qdrant cluster URL")
    QDRANT_API_KEY: str = Field(..., description="Qdrant API key")

    # Authentication
    JWT_SECRET: str = Field(..., description="JWT signing secret")
    JWT_ALGORITHM: str = Field(default="HS256", description="JWT algorithm")

    # Application
    FRONTEND_URL: str = Field(
        default="https://mohsin-raza-developer.github.io",
        description="Frontend origin for CORS"
    )
    LOG_LEVEL: str = Field(default="INFO", description="Logging level")
    ENVIRONMENT: str = Field(default="development", description="Environment (development/production)")

    # Monitoring (optional)
    SENTRY_DSN: str | None = Field(default=None, description="Sentry DSN for error tracking")

    # Model configuration
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True,
        extra="ignore"
    )

    @validator("DATABASE_URL")
    def validate_database_url(cls, v):
        """Ensure DATABASE_URL uses async driver."""
        if not v.startswith("postgresql+asyncpg://"):
            raise ValueError(
                "DATABASE_URL must use asyncpg driver: postgresql+asyncpg://..."
            )
        return v

    @validator("LOG_LEVEL")
    def validate_log_level(cls, v):
        """Validate logging level."""
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if v.upper() not in valid_levels:
            raise ValueError(f"LOG_LEVEL must be one of {valid_levels}")
        return v.upper()


# Global settings instance
settings = Settings()
