"""
Request Validation Middleware

Validates and sanitizes incoming requests for security.
"""

from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
import logging
import re

logger = logging.getLogger(__name__)


class RequestValidationMiddleware(BaseHTTPMiddleware):
    """
    Middleware for request validation and security checks.

    Validates:
    - Content-Type headers
    - Request body size
    - Suspicious patterns (XSS, SQL injection attempts)
    """

    # Maximum request body size (10 MB)
    MAX_BODY_SIZE = 10 * 1024 * 1024

    # Suspicious patterns for basic XSS/injection detection
    SUSPICIOUS_PATTERNS = [
        r'<script[^>]*>.*?</script>',  # Script tags
        r'javascript:',  # JavaScript protocol
        r'on\w+\s*=',  # Event handlers (onclick, onerror, etc.)
        r'<iframe',  # Iframes
        r'UNION\s+SELECT',  # SQL injection pattern
        r'DROP\s+TABLE',  # SQL injection pattern
        r'exec\s*\(',  # Command injection
        r'eval\s*\(',  # Code evaluation
    ]

    def __init__(self, app):
        super().__init__(app)
        self.pattern_regex = re.compile(
            '|'.join(self.SUSPICIOUS_PATTERNS),
            re.IGNORECASE | re.DOTALL
        )

    def _check_suspicious_content(self, content: str) -> bool:
        """
        Check if content contains suspicious patterns.

        Args:
            content: String content to check

        Returns:
            bool: True if suspicious patterns found, False otherwise
        """
        return bool(self.pattern_regex.search(content))

    async def dispatch(self, request: Request, call_next):
        """
        Process request through validation middleware.

        Args:
            request: FastAPI request object
            call_next: Next middleware in chain

        Returns:
            Response object
        """
        # Skip validation for GET requests and health checks
        if request.method == "GET" or request.url.path in ["/health", "/", "/docs", "/openapi.json", "/redoc"]:
            return await call_next(request)

        # Validate Content-Type for POST/PUT/PATCH requests
        if request.method in ["POST", "PUT", "PATCH"]:
            content_type = request.headers.get("content-type", "")

            # Allow JSON and form data
            allowed_types = ["application/json", "application/x-www-form-urlencoded", "multipart/form-data"]

            if not any(allowed in content_type.lower() for allowed in allowed_types):
                logger.warning(
                    f"Invalid Content-Type: {content_type}",
                    extra={"path": request.url.path, "method": request.method}
                )
                return JSONResponse(
                    status_code=415,
                    content={"detail": "Unsupported Media Type. Use application/json."}
                )

        # Check request body size
        if request.method in ["POST", "PUT", "PATCH"]:
            content_length = request.headers.get("content-length")

            if content_length:
                content_length_int = int(content_length)
                if content_length_int > self.MAX_BODY_SIZE:
                    logger.warning(
                        f"Request body too large: {content_length_int} bytes",
                        extra={"path": request.url.path, "method": request.method}
                    )
                    return JSONResponse(
                        status_code=413,
                        content={"detail": f"Request body too large. Maximum size: {self.MAX_BODY_SIZE} bytes."}
                    )

        # For JSON requests, check for suspicious patterns
        if "application/json" in request.headers.get("content-type", "").lower():
            try:
                # Read and decode body
                body = await request.body()
                body_str = body.decode('utf-8')

                # Check for suspicious patterns
                if self._check_suspicious_content(body_str):
                    logger.warning(
                        "Suspicious content detected in request",
                        extra={
                            "path": request.url.path,
                            "method": request.method,
                            "body_preview": body_str[:200]
                        }
                    )
                    return JSONResponse(
                        status_code=400,
                        content={"detail": "Invalid request content detected."}
                    )

                # Reconstruct request with body (since we consumed it)
                async def receive():
                    return {"type": "http.request", "body": body}

                request._receive = receive

            except Exception as e:
                logger.error(f"Error validating request body: {e}")
                return JSONResponse(
                    status_code=400,
                    content={"detail": "Invalid request body."}
                )

        # Process request
        response = await call_next(request)

        # Add security headers to response
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"

        return response


def add_request_validation_middleware(app):
    """
    Add request validation middleware to FastAPI app.

    Args:
        app: FastAPI application instance
    """
    app.add_middleware(RequestValidationMiddleware)
    logger.info("Request validation middleware enabled")
