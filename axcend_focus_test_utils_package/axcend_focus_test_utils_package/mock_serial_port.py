from unittest.mock import Mock
import pytest
import time
from serial import SerialTimeoutException, SerialException

def create_mock_serial_port():
    """Create a mock serial port."""
    mock_port = Mock()

    mock_port.write_data = []
    mock_port.read_data = []
    mock_port.start_time = None

    mock_port.enable_error = False

    def mock_readline() -> bytes:
        """Read data from the serial read buffer."""
        if mock_port.enable_error:
            mock_port.enable_error = False
            raise SerialException
        
        if not mock_port.read_data:
            if mock_port.start_time is None:
                mock_port.start_time = time.time()
            elif time.time() - mock_port.start_time > 0.5:
                raise SerialTimeoutException
            return b""
        else:
            mock_port.start_time = None
            return mock_port.read_data.pop(0)

    def mock_write(data: bytes):
        """Add data to the serial write buffer."""
        mock_port.write_data.append(data)

    def add_to_read_buffer(data: bytes):
        """Add data to the serial read buffer."""
        mock_port.read_data.append(data)

    def error():
        """Raise an error."""
        mock_port.enable_error = True

    mock_port.readline = mock_readline
    mock_port.write = mock_write
    mock_port.close = Mock()  # Add a close method
    mock_port.add_to_read_buffer = add_to_read_buffer
    mock_port.error = error

    return mock_port

@pytest.fixture
def mock_serial_port():
    """Mock the serial port for testing."""
    yield create_mock_serial_port()