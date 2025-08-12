"""
Basic pytest configuration for ROS MCP Server tests.
"""
import pytest
from unittest.mock import Mock, patch
from utils.websocket_manager import WebSocketManager


@pytest.fixture
def shared_websocket_manager():
    """Create a WebSocketManager for testing."""
    return WebSocketManager("127.0.0.1", 9090, "127.0.0.1")


@pytest.fixture
def shared_websocket():
    """Create a mocked websocket library."""
    with patch('utils.websocket_manager.websocket') as mock:
        yield mock



