"""
Environment check script.
Validates all prerequisites before running setup.
"""

import sys
import os
from pathlib import Path

# Ensure UTF-8 encoding for Windows console
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def check_python_version():
    """Check Python version is 3.11+."""
    version = sys.version_info
    if version.major >= 3 and version.minor >= 11:
        print(f"✅ Python {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(f"❌ Python {version.major}.{version.minor}.{version.micro} (requires 3.11+)")
        return False


def check_env_file():
    """Check .env file exists."""
    env_file = Path(__file__).parent.parent / ".env"
    if env_file.exists():
        print(f"✅ .env file found: {env_file}")
        return True
    else:
        print(f"❌ .env file not found: {env_file}")
        print("   Run: cp .env.example .env")
        return False


def check_env_vars():
    """Check environment variables are configured."""
    try:
        from utils.config import settings

        checks = []

        # OpenAI
        if settings.openai_api_key and not settings.openai_api_key.startswith("your-"):
            print(f"✅ OPENAI_API_KEY: {settings.openai_api_key[:10]}...{settings.openai_api_key[-5:]}")
            checks.append(True)
        else:
            print("❌ OPENAI_API_KEY not configured")
            print("   Get key: https://platform.openai.com/api-keys")
            checks.append(False)

        # Qdrant
        if settings.qdrant_url and not settings.qdrant_url.startswith("your-"):
            print(f"✅ QDRANT_URL: {settings.qdrant_url}")
            checks.append(True)
        else:
            print("❌ QDRANT_URL not configured")
            print("   Sign up: https://cloud.qdrant.io/")
            checks.append(False)

        if settings.qdrant_api_key and not settings.qdrant_api_key.startswith("your-"):
            print(f"✅ QDRANT_API_KEY: {settings.qdrant_api_key[:10]}...{settings.qdrant_api_key[-5:]}")
            checks.append(True)
        else:
            print("❌ QDRANT_API_KEY not configured")
            checks.append(False)

        # Neon
        if settings.neon_database_url and "postgresql://" in settings.neon_database_url:
            # Mask password
            import re
            masked_url = re.sub(r'://([^:]+):([^@]+)@', r'://\1:***@', settings.neon_database_url)
            print(f"✅ NEON_DATABASE_URL: {masked_url}")
            checks.append(True)
        else:
            print("❌ NEON_DATABASE_URL not configured")
            print("   Sign up: https://neon.tech/")
            checks.append(False)

        return all(checks)

    except Exception as e:
        print(f"❌ Failed to load config: {str(e)}")
        return False


def check_dependencies():
    """Check required Python packages are installed."""
    required_packages = [
        "fastapi",
        "openai",
        "qdrant_client",
        "asyncpg",
        "pydantic",
        "uvicorn"
    ]

    checks = []

    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package} installed")
            checks.append(True)
        except ImportError:
            print(f"❌ {package} not installed")
            checks.append(False)

    if not all(checks):
        print("\n   Run: pip install -r requirements.txt")

    return all(checks)


def check_book_content():
    """Check book MDX files exist."""
    docs_dir = Path(__file__).parent.parent.parent / "humanoid_robot_book" / "docs"

    if not docs_dir.exists():
        print(f"❌ Docs directory not found: {docs_dir}")
        return False

    mdx_files = list(docs_dir.glob("*.mdx"))

    if not mdx_files:
        print(f"⚠️  No MDX files found in {docs_dir}")
        print("   You can still run setup, but embedding will be skipped")
        return True

    print(f"✅ Found {len(mdx_files)} MDX files in {docs_dir}")
    for f in sorted(mdx_files)[:5]:  # Show first 5
        print(f"   - {f.name}")
    if len(mdx_files) > 5:
        print(f"   ... and {len(mdx_files) - 5} more")

    return True


def main():
    """Run all environment checks."""

    print("=" * 60)
    print("RAG Chatbot Backend - Environment Check")
    print("=" * 60)
    print()

    checks = []

    print("Checking Python version...")
    checks.append(check_python_version())
    print()

    print("Checking .env file...")
    checks.append(check_env_file())
    print()

    print("Checking environment variables...")
    checks.append(check_env_vars())
    print()

    print("Checking dependencies...")
    checks.append(check_dependencies())
    print()

    print("Checking book content...")
    checks.append(check_book_content())
    print()

    print("=" * 60)

    if all(checks):
        print("✅ All checks passed!")
        print("=" * 60)
        print()
        print("You're ready to run setup:")
        print("  python scripts/setup_all.py")
        print()
        print("Or run setup steps manually:")
        print("  1. python scripts/run_migrations.py migrate")
        print("  2. python scripts/setup_qdrant.py")
        print("  3. python scripts/embed_book_content.py")
        print("  4. python src/main.py")
        print()
        return 0

    else:
        print("❌ Some checks failed")
        print("=" * 60)
        print()
        print("Please fix the issues above before running setup.")
        print("See backend/SETUP_CHECKLIST.md for detailed instructions.")
        print()
        return 1


if __name__ == "__main__":
    sys.exit(main())
