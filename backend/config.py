"""
Configuration module for RAG Chatbot Backend API.

Loads environment variables using pydantic-settings for type-safe configuration.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""

    # LLM Configuration
    gemini_api_key: str = Field(
        ...,
        description="Google Gemini API key for LLM inference"
    )

    openai_api_key: str = Field(
        ...,
        description="OpenAI API key for LLM inference"
    )

    # Embeddings Configuration
    cohere_api_key: str = Field(
        ...,
        description="Cohere API key for text embeddings"
    )

    # Vector Database Configuration
    qdrant_url: str = Field(
        ...,
        description="Qdrant Cloud instance URL"
    )
    qdrant_api_key: str = Field(
        ...,
        description="Qdrant API key for authentication"
    )



    # Security Configuration
    jwt_secret: str = Field(
        ...,
        description="Secret key for JWT token validation"
    )

    # Rate Limiting Configuration
    rate_limit_per_minute: int = Field(
        default=20,
        description="Maximum requests per minute per user"
    )



    # Qdrant Configuration
    qdrant_collection_name: str = Field(
        default="robotics_textbook_v1",
        description="Qdrant collection name for textbook embeddings"
    )
    qdrant_timeout: float = Field(
        default=10.0,
        description="Qdrant client timeout in seconds"
    )

    # Session Configuration
    session_timeout_hours: int = Field(
        default=1,
        description="Session inactivity timeout in hours"
    )
    max_tokens_per_session: int = Field(
        default=8000,
        description="Maximum tokens in conversation context window"
    )

    # CORS Configuration
    cors_origins: list[str] = Field(
        default=["http://localhost:3000", "http://localhost:5173"],
        description="Allowed CORS origins (comma-separated in .env)"
    )

    # Logging Configuration
    log_level: str = Field(
        default="INFO",
        description="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)"
    )

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )


# Global settings instance
settings = Settings()
