"""
Tests for the WebSocketManager class.
"""
import pytest
from unittest.mock import Mock


class TestWebSocketManager:
    """Test cases for WebSocketManager class."""

    def test_connect_success(self, shared_websocket_manager, shared_websocket):
        """Test successful WebSocket connection."""
        # Setup mock
        mock_ws = Mock()
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.connected = True

        # Test connection
        result = shared_websocket_manager.connect()

        # Verify results
        assert result is None  # connect() returns None on success
        assert shared_websocket_manager.ws == mock_ws
        shared_websocket.create_connection.assert_called_once_with(
            "ws://127.0.0.1:9090", timeout=2.0
        )

    def test_connect_failure(self, shared_websocket_manager, shared_websocket):
        """Test WebSocket connection failure."""
        # Setup mock to raise exception
        shared_websocket.create_connection.side_effect = Exception("Connection failed")

        # Test connection
        result = shared_websocket_manager.connect()

        # Verify results
        assert result is not None
        assert "Connection failed" in result
        assert shared_websocket_manager.ws is None

    def test_connect_already_connected(self, shared_websocket_manager, shared_websocket):
        """Test connection when already connected."""
        # Setup existing connection
        mock_ws = Mock()
        mock_ws.connected = True

        shared_websocket_manager.ws = mock_ws
        result = shared_websocket_manager.connect()

        # Should return None and not create new connection
        assert result is None
        shared_websocket.create_connection.assert_not_called()

    def test_set_ip(self, shared_websocket_manager):
        """Test setting IP and port."""
        # Test setting new IP and port
        shared_websocket_manager.set_ip("192.168.1.100", 8080, "192.168.1.101")
        
        assert shared_websocket_manager.ip == "192.168.1.100"
        assert shared_websocket_manager.port == 8080
        assert shared_websocket_manager.local_ip == "192.168.1.101"

    def test_set_ip_without_local_ip(self, shared_websocket_manager):
        """Test setting IP without changing local_ip."""
        original_local_ip = shared_websocket_manager.local_ip
        shared_websocket_manager.set_ip("192.168.1.100", 8080)
        
        assert shared_websocket_manager.ip == "192.168.1.100"
        assert shared_websocket_manager.port == 8080
        assert shared_websocket_manager.local_ip == original_local_ip

    def test_send_success(self, shared_websocket_manager, shared_websocket):
        """Test successful message sending."""
        # Setup mock websocket
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        
        # Test sending a message
        message = {"op": "subscribe", "topic": "/cmd_vel"}
        result = shared_websocket_manager.send(message)
        
        # Verify results
        assert result is None  # send() returns None on success
        mock_ws.send.assert_called_once_with('{"op": "subscribe", "topic": "/cmd_vel"}')

    def test_send_connection_failure(self, shared_websocket_manager, shared_websocket):
        """Test message sending with connection failure."""
        # Setup mock to raise connection error
        shared_websocket.create_connection.side_effect = Exception("Connection failed")
        
        message = {"op": "subscribe", "topic": "/cmd_vel"}
        result = shared_websocket_manager.send(message)
        
        # Verify results
        assert result is not None
        assert "Connection failed" in result

    def test_send_json_error(self, shared_websocket_manager, shared_websocket):
        """Test message sending with JSON serialization error."""
        # Setup mock websocket
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        
        # Create a non-serializable object
        non_serializable = Mock()
        message = {"op": "subscribe", "topic": "/cmd_vel", "data": non_serializable}
        
        result = shared_websocket_manager.send(message)
        
        # Verify results
        assert result is not None
        assert "JSON serialization error" in result

    def test_send_websocket_error(self, shared_websocket_manager, shared_websocket):
        """Test message sending with WebSocket error."""
        # Setup mock websocket that raises error on send
        mock_ws = Mock()
        mock_ws.connected = True
        mock_ws.send.side_effect = Exception("Send failed")
        shared_websocket.create_connection.return_value = mock_ws
        
        message = {"op": "subscribe", "topic": "/cmd_vel"}
        result = shared_websocket_manager.send(message)
        
        # Verify results
        assert result is not None
        assert "Send error" in result

    def test_receive_success(self, shared_websocket_manager, shared_websocket):
        """Test successful message receiving."""
        # Setup mock websocket
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.recv.return_value = '{"op": "publish", "topic": "/cmd_vel"}'
        
        # Test receiving a message
        result = shared_websocket_manager.receive(timeout=1.0)
        
        # Verify results
        assert result == '{"op": "publish", "topic": "/cmd_vel"}'
        mock_ws.settimeout.assert_called_once_with(1.0)

    def test_receive_timeout(self, shared_websocket_manager, shared_websocket):
        """Test message receiving with timeout."""
        # Setup mock websocket that raises timeout
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.recv.side_effect = Exception("Timeout")
        
        result = shared_websocket_manager.receive(timeout=1.0)
        
        # Verify results
        assert result is None

    def test_request_success(self, shared_websocket_manager, shared_websocket):
        """Test successful request-response cycle."""
        # Setup mock websocket
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.recv.return_value = '{"op": "service_response", "values": {"topics": ["/cmd_vel"]}}'
        
        # Test request
        message = {"op": "call_service", "service": "/rosapi/topics"}
        result = shared_websocket_manager.request(message)
        
        # Verify results
        expected = {"op": "service_response", "values": {"topics": ["/cmd_vel"]}}
        assert result == expected

    def test_request_connection_failure(self, shared_websocket_manager, shared_websocket):
        """Test request with connection failure."""
        # Setup mock to raise connection error
        shared_websocket.create_connection.side_effect = Exception("Connection failed")
        
        message = {"op": "call_service", "service": "/rosapi/topics"}
        result = shared_websocket_manager.request(message)
        
        # Verify results
        assert "error" in result
        assert "Connection failed" in result["error"]

    def test_request_send_failure(self, shared_websocket_manager, shared_websocket):
        """Test request with send failure."""
        # Setup mock websocket that fails on send
        mock_ws = Mock()
        mock_ws.connected = True
        mock_ws.send.side_effect = Exception("Send failed")
        shared_websocket.create_connection.return_value = mock_ws
        
        message = {"op": "call_service", "service": "/rosapi/topics"}
        result = shared_websocket_manager.request(message)
        
        # Verify results
        assert "error" in result
        assert "Send failed" in result["error"]

    def test_request_receive_failure(self, shared_websocket_manager, shared_websocket):
        """Test request with receive failure."""
        # Setup mock websocket that fails on receive
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.recv.return_value = None  # No response
        
        message = {"op": "call_service", "service": "/rosapi/topics"}
        result = shared_websocket_manager.request(message)
        
        # Verify results
        assert "error" in result
        assert "no response or timeout" in result["error"]

    def test_request_invalid_json(self, shared_websocket_manager, shared_websocket):
        """Test request with invalid JSON response."""
        # Setup mock websocket that returns invalid JSON
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket.create_connection.return_value = mock_ws
        mock_ws.recv.return_value = "invalid json"
        
        message = {"op": "call_service", "service": "/rosapi/topics"}
        result = shared_websocket_manager.request(message)
        
        # Verify results
        assert "error" in result
        assert result["error"] == "invalid_json"
        assert result["raw"] == "invalid json"

    def test_close_success(self, shared_websocket_manager):
        """Test successful WebSocket closure."""
        # Setup existing connection
        mock_ws = Mock()
        mock_ws.connected = True
        shared_websocket_manager.ws = mock_ws
        
        # Test closing
        shared_websocket_manager.close()
        
        # Verify results
        mock_ws.close.assert_called_once()
        assert shared_websocket_manager.ws is None

    def test_close_failure(self, shared_websocket_manager):
        """Test WebSocket closure with error."""
        # Setup existing connection that fails to close
        mock_ws = Mock()
        mock_ws.connected = True
        mock_ws.close.side_effect = Exception("Close failed")
        shared_websocket_manager.ws = mock_ws
        
        # Test closing (should not raise exception)
        shared_websocket_manager.close()
        
        # Verify results - should still set ws to None even if close fails
        assert shared_websocket_manager.ws is None

    def test_close_not_connected(self, shared_websocket_manager):
        """Test closing when not connected."""
        # Ensure no connection exists
        shared_websocket_manager.ws = None
        
        # Test closing (should not raise exception)
        shared_websocket_manager.close()
        
        # Verify results
        assert shared_websocket_manager.ws is None
