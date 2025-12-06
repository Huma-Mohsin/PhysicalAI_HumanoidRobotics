"""Initial schema

Revision ID: 001
Revises:
Create Date: 2025-12-06

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create users table
    op.create_table(
        'users',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), nullable=False, unique=True),
        sa.Column('password_hash', sa.String(255), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('is_active', sa.Boolean(), default=True, nullable=False),
    )
    op.create_index('idx_users_email', 'users', ['email'])

    # Create hardware_profiles table
    op.create_table(
        'hardware_profiles',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False, unique=True),
        sa.Column('os_type', sa.String(50), nullable=False),
        sa.Column('gpu_model', sa.String(100), nullable=True),
        sa.Column('environment_preference', sa.Enum('workstation', 'cloud', 'mac', name='environmentpreference'), nullable=False),
        sa.Column('language_preference', sa.Enum('en', 'ur', name='languagepreference'), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
    )
    op.create_index('idx_hardware_profiles_user_id', 'hardware_profiles', ['user_id'])

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=True),
        sa.Column('query', sa.Text(), nullable=False),
        sa.Column('response', sa.Text(), nullable=False),
        sa.Column('selected_text', sa.Text(), nullable=True),
        sa.Column('language', sa.String(2), nullable=False),
        sa.Column('session_id', sa.String(255), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
    )
    op.create_index('idx_chat_messages_user_id', 'chat_messages', ['user_id'])

    # Create feedback table
    op.create_table(
        'feedback',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True),
        sa.Column('message_id', postgresql.UUID(as_uuid=True), nullable=False, unique=True),
        sa.Column('rating', sa.Enum('thumbs_up', 'thumbs_down', name='feedbackrating'), nullable=False),
        sa.Column('optional_text', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['message_id'], ['chat_messages.id'], ondelete='CASCADE'),
    )
    op.create_index('idx_feedback_message_id', 'feedback', ['message_id'])


def downgrade() -> None:
    op.drop_index('idx_feedback_message_id', table_name='feedback')
    op.drop_table('feedback')
    op.drop_index('idx_chat_messages_user_id', table_name='chat_messages')
    op.drop_table('chat_messages')
    op.drop_index('idx_hardware_profiles_user_id', table_name='hardware_profiles')
    op.drop_table('hardware_profiles')
    op.drop_index('idx_users_email', table_name='users')
    op.drop_table('users')

    # Drop enum types
    op.execute('DROP TYPE IF EXISTS feedbackrating')
    op.execute('DROP TYPE IF EXISTS languagepreference')
    op.execute('DROP TYPE IF EXISTS environmentpreference')
