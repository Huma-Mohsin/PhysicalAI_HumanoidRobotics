"""
Integration tests for chat and RAG endpoints.
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock


# ============================================================================
# Integration Tests - Chat Endpoints
# ============================================================================

@pytest.mark.integration
@pytest.mark.rag
class TestChatEndpoints:
    """Test chat query API endpoints."""

    @patch('src.services.rag_service.RAGService.process_query')
    def test_chat_query_success(
        self,
        mock_process_query: MagicMock,
        client: TestClient,
        mock_chat_query
    ):
        """Test successful chat query processing."""
        # Mock RAG service response
        mock_process_query.return_value = {
            "response": "To create a ROS 2 node, you need to...",
            "retrieved_chunks": [
                {
                    "id": "chunk-1",
                    "score": 0.9,
                    "payload": {
                        "content": "ROS 2 node creation guide...",
                        "metadata": {"module": "ros2", "chapter": "basics"}
                    }
                }
            ],
            "num_chunks": 1
        }

        response = client.post("/chat", json=mock_chat_query)

        assert response.status_code == 200
        data = response.json()

        # Check response structure
        assert "response" in data
        assert "retrieved_chunks" in data
        assert "num_chunks" in data
        assert "language" in data
        assert "timestamp" in data

        # Verify RAG service was called
        mock_process_query.assert_called_once()

    @patch('src.services.rag_service.RAGService.process_query')
    def test_chat_query_with_context(
        self,
        mock_process_query: MagicMock,
        client: TestClient
    ):
        """Test chat query with selected text context."""
        query_with_context = {
            "query": "Explain this concept",
            "language": "en",
            "selected_text": "ROS 2 uses DDS for communication between nodes."
        }

        mock_process_query.return_value = {
            "response": "DDS (Data Distribution Service) is...",
            "retrieved_chunks": [],
            "num_chunks": 0
        }

        response = client.post("/chat", json=query_with_context)

        assert response.status_code == 200

        # Verify selected_text was passed to RAG service
        call_args = mock_process_query.call_args
        assert call_args[1]["selected_text"] == query_with_context["selected_text"]

    @patch('src.services.rag_service.RAGService.process_query')
    def test_chat_query_with_history(
        self,
        mock_process_query: MagicMock,
        client: TestClient
    ):
        """Test chat query with conversation history."""
        query_with_history = {
            "query": "And how do I publish messages?",
            "language": "en",
            "chat_history": [
                {"role": "user", "content": "How do I create a ROS 2 node?"},
                {"role": "assistant", "content": "To create a ROS 2 node..."}
            ]
        }

        mock_process_query.return_value = {
            "response": "To publish messages in ROS 2...",
            "retrieved_chunks": [],
            "num_chunks": 0
        }

        response = client.post("/chat", json=query_with_history)

        assert response.status_code == 200

        # Verify history was passed
        call_args = mock_process_query.call_args
        assert call_args[1]["chat_history"] is not None

    def test_chat_query_missing_fields(self, client: TestClient):
        """Test chat query with missing required fields."""
        invalid_query = {
            "language": "en"  # Missing 'query' field
        }

        response = client.post("/chat", json=invalid_query)

        assert response.status_code == 422  # Validation error

    @patch('src.services.rag_service.RAGService.process_query')
    def test_chat_query_urdu_language(
        self,
        mock_process_query: MagicMock,
        client: TestClient
    ):
        """Test chat query in Urdu language."""
        urdu_query = {
            "query": "ROS 2 node کیسے بنائیں؟",
            "language": "ur"
        }

        mock_process_query.return_value = {
            "response": "ROS 2 node بنانے کے لیے...",
            "retrieved_chunks": [],
            "num_chunks": 0
        }

        response = client.post("/chat", json=urdu_query)

        assert response.status_code == 200
        assert response.json()["language"] == "ur"

    def test_chat_health_endpoint(self, client: TestClient):
        """Test chat service health check."""
        response = client.get("/chat/health")

        assert response.status_code == 200
        data = response.json()

        assert "status" in data
        assert "rag_service" in data
        assert "timestamp" in data


# ============================================================================
# Unit Tests - Chat Schemas
# ============================================================================

@pytest.mark.unit
@pytest.mark.rag
class TestChatSchemas:
    """Test chat request/response schemas."""

    def test_chat_request_validation(self):
        """Test ChatRequest schema validation."""
        from src.schemas.chat import ChatRequest

        # Valid request
        valid_data = {
            "query": "Test query",
            "language": "en"
        }
        request = ChatRequest(**valid_data)
        assert request.query == "Test query"
        assert request.language == "en"

        # Invalid language
        with pytest.raises(ValueError):
            ChatRequest(query="Test", language="invalid")

    def test_chat_history_message_format(self):
        """Test chat history message structure."""
        from src.schemas.chat import ChatHistoryMessage

        # Valid message
        message = ChatHistoryMessage(role="user", content="Hello")
        assert message.role == "user"
        assert message.content == "Hello"

        # Invalid role
        with pytest.raises(ValueError):
            ChatHistoryMessage(role="invalid", content="Hello")
