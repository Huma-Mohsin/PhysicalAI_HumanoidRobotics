"""
Migration Runner
Runs SQL migration files against Neon PostgreSQL database.
"""

import asyncio
import asyncpg
from pathlib import Path
from src.utils.config import settings


async def run_migration(migration_file: str):
    """Run a specific migration file."""
    print(f"Running migration: {migration_file}")

    # Read migration SQL
    migration_path = Path(__file__).parent / "database" / "migrations" / migration_file

    if not migration_path.exists():
        print(f"[ERROR] Migration file not found: {migration_path}")
        return False

    with open(migration_path, "r") as f:
        sql = f.read()

    # Connect to database
    conn = await asyncpg.connect(settings.neon_database_url)

    try:
        # Run migration
        print(f"Executing SQL from {migration_file}...")
        await conn.execute(sql)
        print(f"[SUCCESS] Migration {migration_file} completed successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Migration failed: {str(e)}")
        return False
    finally:
        await conn.close()


async def main():
    """Run the latest migration."""
    success = await run_migration("004_add_background_fields.sql")

    if success:
        print("\n[SUCCESS] Migration 004 completed successfully!")
        print("New columns added to user_profiles:")
        print("  - software_experience")
        print("  - programming_languages")
        print("  - hardware_type")
        print("  - hardware_experience")
    else:
        print("\n[ERROR] Migration failed. Check logs for details.")


if __name__ == "__main__":
    asyncio.run(main())
