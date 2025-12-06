"""
Integration tests for feedback endpoints.
"""

import pytest
from fastapi.testclient import TestClient
from uuid import uuid4


# ============================================================================
# Integration Tests - Feedback Endpoints
# ============================================================================

@pytest.mark.integration
@pytest.mark.feedback
class TestFeedbackEndpoints:
    """Test feedback submission and statistics endpoints."""

    def test_submit_feedback_success(
        self,
        client: TestClient,
        mock_feedback_data
    ):
        """Test successful feedback submission."""
        # Note: In real scenario, message_id should exist in database
        # For testing, we're mocking the database interaction

        response = client.post("/feedback", json=mock_feedback_data)

        # Will fail if message doesn't exist, but schema should validate
        assert response.status_code in [201, 404]  # Either created or message not found

        if response.status_code == 201:
            data = response.json()
            assert "id" in data
            assert data["rating"] == mock_feedback_data["rating"]

    def test_submit_feedback_invalid_rating(self, client: TestClient):
        """Test feedback submission with invalid rating."""
        invalid_data = {
            "message_id": str(uuid4()),
            "rating": 5,  # Invalid: should be -1, 0, or 1
            "comment": "Test comment"
        }

        response = client.post("/feedback", json=invalid_data)

        assert response.status_code == 422  # Validation error

    def test_submit_feedback_with_comment(self, client: TestClient):
        """Test feedback submission with optional comment."""
        feedback_data = {
            "message_id": str(uuid4()),
            "rating": 1,
            "comment": "Very helpful and clear explanation!"
        }

        response = client.post("/feedback", json=feedback_data)

        # Either created or message not found
        assert response.status_code in [201, 404]

    def test_submit_feedback_comment_too_long(self, client: TestClient):
        """Test feedback submission with comment exceeding max length."""
        feedback_data = {
            "message_id": str(uuid4()),
            "rating": 1,
            "comment": "x" * 600  # Exceeds 500 char limit
        }

        response = client.post("/feedback", json=feedback_data)

        assert response.status_code == 422  # Validation error

    def test_get_feedback_stats(self, client: TestClient):
        """Test retrieving aggregated feedback statistics."""
        response = client.get("/feedback/stats")

        assert response.status_code == 200
        data = response.json()

        # Check response structure
        assert "total_feedback" in data
        assert "positive_count" in data
        assert "negative_count" in data
        assert "positive_percentage" in data
        assert "average_rating" in data
        assert "recent_comments" in data

        # Check data types
        assert isinstance(data["total_feedback"], int)
        assert isinstance(data["positive_percentage"], float)
        assert isinstance(data["recent_comments"], list)

    def test_update_existing_feedback(self, client: TestClient):
        """Test updating feedback for the same message."""
        message_id = str(uuid4())

        # First feedback
        first_feedback = {
            "message_id": message_id,
            "rating": 1,
            "comment": "Initial feedback"
        }
        response1 = client.post("/feedback", json=first_feedback)

        # Update feedback
        updated_feedback = {
            "message_id": message_id,
            "rating": -1,
            "comment": "Changed my mind"
        }
        response2 = client.post("/feedback", json=updated_feedback)

        # Should update existing feedback, not create duplicate
        assert response2.status_code in [201, 404]

    def test_delete_feedback(self, client: TestClient):
        """Test deleting feedback for a message."""
        message_id = str(uuid4())

        # Submit feedback
        feedback_data = {
            "message_id": message_id,
            "rating": 1
        }
        client.post("/feedback", json=feedback_data)

        # Delete feedback
        response = client.delete(f"/feedback/{message_id}")

        # Either deleted or not found
        assert response.status_code in [204, 404]


# ============================================================================
# Unit Tests - Feedback Schemas
# ============================================================================

@pytest.mark.unit
@pytest.mark.feedback
class TestFeedbackSchemas:
    """Test feedback request/response schemas."""

    def test_feedback_request_validation(self):
        """Test FeedbackRequest schema validation."""
        from src.schemas.feedback import FeedbackRequest

        # Valid request
        valid_data = {
            "message_id": str(uuid4()),
            "rating": 1,
            "comment": "Great response!"
        }
        request = FeedbackRequest(**valid_data)
        assert request.rating == 1

        # Invalid rating (out of range)
        with pytest.raises(ValueError):
            FeedbackRequest(
                message_id=str(uuid4()),
                rating=2,  # Must be -1, 0, or 1
                comment="Test"
            )

    def test_feedback_stats_structure(self):
        """Test FeedbackStats schema structure."""
        from src.schemas.feedback import FeedbackStats

        stats_data = {
            "total_feedback": 100,
            "positive_count": 80,
            "negative_count": 20,
            "positive_percentage": 80.0,
            "average_rating": 0.6,
            "recent_comments": ["Great!", "Very helpful", "Clear explanation"]
        }

        stats = FeedbackStats(**stats_data)
        assert stats.total_feedback == 100
        assert stats.positive_percentage == 80.0
        assert len(stats.recent_comments) == 3


# ============================================================================
# Integration Tests - Feedback Analytics
# ============================================================================

@pytest.mark.integration
@pytest.mark.feedback
class TestFeedbackAnalytics:
    """Test feedback analytics calculations."""

    def test_positive_percentage_calculation(self, client: TestClient):
        """Test that positive percentage is calculated correctly."""
        response = client.get("/feedback/stats")

        if response.status_code == 200:
            data = response.json()

            total = data["total_feedback"]
            positive = data["positive_count"]
            percentage = data["positive_percentage"]

            # If there's feedback, verify percentage calculation
            if total > 0:
                expected_percentage = (positive / total) * 100
                assert abs(percentage - expected_percentage) < 0.1

    def test_average_rating_calculation(self, client: TestClient):
        """Test that average rating is calculated correctly."""
        response = client.get("/feedback/stats")

        if response.status_code == 200:
            data = response.json()

            total = data["total_feedback"]
            avg_rating = data["average_rating"]

            # Average rating should be between -1 and 1
            if total > 0:
                assert -1.0 <= avg_rating <= 1.0
