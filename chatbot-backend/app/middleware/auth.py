"""
JWT authentication middleware and dependencies.

Provides user authentication via JWT token validation.
"""
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt
from app.config import settings
from app.utils.errors import AuthenticationError

# Security scheme for JWT bearer token
security = HTTPBearer()


def decode_jwt_token(token: str) -> dict:
    """
    Decode and validate JWT token.

    Args:
        token: JWT token string

    Returns:
        dict: Decoded token payload

    Raises:
        AuthenticationError: If token is invalid or expired
    """
    try:
        payload = jwt.decode(
            token,
            settings.JWT_SECRET,
            algorithms=[settings.JWT_ALGORITHM]
        )
        return payload
    except jwt.ExpiredSignatureError:
        raise AuthenticationError("Token has expired")
    except jwt.InvalidTokenError:
        raise AuthenticationError("Invalid authentication token")


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> str:
    """
    Dependency to extract and validate current user from JWT token.

    Usage in routes:
        @app.get("/protected")
        async def protected_route(user_id: str = Depends(get_current_user)):
            # user_id is automatically extracted from token
            return {"user_id": user_id}

    Args:
        credentials: HTTP Authorization credentials

    Returns:
        str: User ID extracted from token

    Raises:
        HTTPException: If authentication fails
    """
    try:
        token = credentials.credentials
        payload = decode_jwt_token(token)

        # Extract user_id from payload
        user_id = payload.get("sub") or payload.get("user_id") or payload.get("id")

        if not user_id:
            raise AuthenticationError("Token does not contain user identifier")

        return user_id

    except AuthenticationError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e),
            headers={"WWW-Authenticate": "Bearer"},
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication failed",
            headers={"WWW-Authenticate": "Bearer"},
        )
