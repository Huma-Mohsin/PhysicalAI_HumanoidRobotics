"""
Database migration script for Neon Postgres.
Executes SQL migration files against the database.
"""

import asyncio
import asyncpg
import sys
from pathlib import Path

# Add parent directory to path to import utils
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from utils.config import settings
from utils.logger import logger


async def run_migration(migration_file: Path):
    """
    Execute a SQL migration file against the database.

    Args:
        migration_file: Path to the SQL migration file

    Raises:
        Exception: If migration fails
    """
    logger.info(f"Running migration: {migration_file.name}")

    # Read migration SQL
    with open(migration_file, "r") as f:
        sql = f.read()

    # Connect to database
    conn = await asyncpg.connect(settings.neon_database_url)

    try:
        # Execute migration in a transaction
        async with conn.transaction():
            await conn.execute(sql)
        logger.info(f"✅ Migration {migration_file.name} completed successfully")
    except Exception as e:
        logger.error(f"❌ Migration {migration_file.name} failed: {str(e)}")
        raise
    finally:
        await conn.close()


async def run_all_migrations():
    """Run all pending migrations in order."""
    migrations_dir = Path(__file__).parent.parent / "migrations"

    # Get all migration files (exclude rollback files)
    migration_files = sorted(
        [f for f in migrations_dir.glob("*.sql") if "rollback" not in f.name]
    )

    if not migration_files:
        logger.warning("No migration files found")
        return

    logger.info(f"Found {len(migration_files)} migration(s)")

    for migration_file in migration_files:
        await run_migration(migration_file)

    logger.info("🎉 All migrations completed successfully")


async def rollback_migration(migration_number: str):
    """
    Rollback a specific migration.

    Args:
        migration_number: Migration number (e.g., "001")
    """
    migrations_dir = Path(__file__).parent.parent / "migrations"
    rollback_file = migrations_dir / f"{migration_number}_rollback.sql"

    if not rollback_file.exists():
        logger.error(f"Rollback file not found: {rollback_file}")
        return

    logger.info(f"Rolling back migration {migration_number}")

    # Read rollback SQL
    with open(rollback_file, "r") as f:
        sql = f.read()

    # Connect to database
    conn = await asyncpg.connect(settings.neon_database_url)

    try:
        # Execute rollback in a transaction
        async with conn.transaction():
            await conn.execute(sql)
        logger.info(f"✅ Rollback {migration_number} completed successfully")
    except Exception as e:
        logger.error(f"❌ Rollback {migration_number} failed: {str(e)}")
        raise
    finally:
        await conn.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Database migration management")
    parser.add_argument(
        "action",
        choices=["migrate", "rollback"],
        help="Action to perform"
    )
    parser.add_argument(
        "--migration",
        type=str,
        help="Migration number for rollback (e.g., 001)"
    )

    args = parser.parse_args()

    if args.action == "migrate":
        asyncio.run(run_all_migrations())
    elif args.action == "rollback":
        if not args.migration:
            logger.error("Please specify --migration number for rollback")
            sys.exit(1)
        asyncio.run(rollback_migration(args.migration))
