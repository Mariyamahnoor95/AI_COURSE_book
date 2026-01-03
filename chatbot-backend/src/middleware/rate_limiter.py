"""
Rate limiting middleware for chatbot API

Implements per-user rate limiting to prevent abuse
Maps to: FR-021, SC-006
"""

from typing import Dict, Tuple, Optional
from datetime import datetime, timedelta
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
import hashlib
import time


class RateLimiter:
    """
    In-memory rate limiter for chatbot queries

    Limits:
    - 10 queries per hour per user (identified by IP address)
    - Returns 429 status code when exceeded
    - Provides time-until-reset in error message

    Note: For production, use Redis-backed rate limiting
    to support multiple backend instances
    """

    def __init__(self, max_requests: int = 10, window_hours: int = 1):
        """
        Initialize rate limiter

        Args:
            max_requests: Maximum requests allowed per window (default: 10)
            window_hours: Time window in hours (default: 1)
        """
        self.max_requests = max_requests
        self.window_seconds = window_hours * 3600

        # In-memory store: {user_id: [(timestamp, timestamp, ...)]}
        # Each user_id maps to a list of request timestamps
        self._request_history: Dict[str, list[float]] = {}

    def _get_user_identifier(self, request: Request) -> str:
        """
        Generate unique identifier for user

        Uses IP address for anonymous users. In production with auth,
        this should use user_id from authentication token.

        Args:
            request: FastAPI request object

        Returns:
            Hashed user identifier
        """
        # Get IP address from request
        # Check for forwarded IP (e.g., behind proxy/load balancer)
        forwarded_for = request.headers.get("X-Forwarded-For")
        if forwarded_for:
            client_ip = forwarded_for.split(",")[0].strip()
        else:
            client_ip = request.client.host if request.client else "unknown"

        # Hash the IP for privacy
        return hashlib.sha256(client_ip.encode()).hexdigest()[:16]

    def _cleanup_old_requests(self, user_id: str, current_time: float) -> None:
        """
        Remove timestamps outside the current time window

        Args:
            user_id: User identifier
            current_time: Current Unix timestamp
        """
        if user_id not in self._request_history:
            return

        # Keep only requests within the time window
        cutoff_time = current_time - self.window_seconds
        self._request_history[user_id] = [
            ts for ts in self._request_history[user_id]
            if ts > cutoff_time
        ]

        # Remove user entry if no recent requests
        if not self._request_history[user_id]:
            del self._request_history[user_id]

    def _get_time_until_reset(self, user_id: str, current_time: float) -> int:
        """
        Calculate minutes until rate limit resets

        Args:
            user_id: User identifier
            current_time: Current Unix timestamp

        Returns:
            Minutes until oldest request expires
        """
        if user_id not in self._request_history or not self._request_history[user_id]:
            return 0

        # Find oldest request timestamp
        oldest_request = min(self._request_history[user_id])
        reset_time = oldest_request + self.window_seconds

        # Calculate minutes until reset
        seconds_remaining = max(0, reset_time - current_time)
        minutes_remaining = int(seconds_remaining / 60) + 1  # Round up

        return minutes_remaining

    def check_rate_limit(self, request: Request) -> Tuple[bool, Optional[str]]:
        """
        Check if request exceeds rate limit

        Args:
            request: FastAPI request object

        Returns:
            Tuple of (is_allowed, error_detail)
            - is_allowed: True if request is allowed, False if rate limited
            - error_detail: Error message if rate limited, None otherwise
        """
        current_time = time.time()
        user_id = self._get_user_identifier(request)

        # Clean up old requests
        self._cleanup_old_requests(user_id, current_time)

        # Get user's request history
        if user_id not in self._request_history:
            self._request_history[user_id] = []

        request_count = len(self._request_history[user_id])

        # Check if rate limit exceeded
        if request_count >= self.max_requests:
            minutes_until_reset = self._get_time_until_reset(user_id, current_time)

            error_detail = (
                f"Maximum {self.max_requests} queries per hour exceeded. "
                f"Try again in {minutes_until_reset} minute{'s' if minutes_until_reset != 1 else ''}."
            )
            return False, error_detail

        # Record this request
        self._request_history[user_id].append(current_time)

        return True, None

    async def __call__(self, request: Request, call_next):
        """
        Middleware handler for FastAPI

        Args:
            request: Incoming HTTP request
            call_next: Next middleware/route handler

        Returns:
            HTTP response (429 if rate limited, otherwise passes through)
        """
        # Only apply rate limiting to /api/chat endpoint
        if request.url.path == "/api/chat" and request.method == "POST":
            is_allowed, error_detail = self.check_rate_limit(request)

            if not is_allowed:
                return JSONResponse(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    content={
                        "error": "Rate limit exceeded",
                        "detail": error_detail
                    }
                )

        # Continue to next handler
        response = await call_next(request)
        return response


# Global rate limiter instance
rate_limiter = RateLimiter(max_requests=10, window_hours=1)


# Convenience function for dependency injection
def get_rate_limiter() -> RateLimiter:
    """Get global rate limiter instance"""
    return rate_limiter
