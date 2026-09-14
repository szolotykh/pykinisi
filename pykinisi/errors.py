# Filename: errors.py
# Description: Public exceptions for protocol, transport, and controller failures.
"""Exceptions retain wire correlation fields for controller error handling."""


class KinisiError(Exception):
    """Base class for SDK communication failures."""


class ConnectionClosedError(KinisiError):
    """The serial connection is closed or failed during an operation."""


class ProtocolError(KinisiError):
    """The peer sent an invalid or incompatible protocol message."""


class RequestTimeoutError(KinisiError, TimeoutError):
    """A reply missed its deadline; the command may have executed."""


class ControllerError(KinisiError):
    """A shared ERROR reply with its failed command and original message ID."""

    def __init__(self, command, message_id, error_code):
        """Decode known error names while preserving codes from newer firmware."""
        from .KinisiCommands import ErrorCode, ERROR_DESCRIPTIONS
        self.command = command
        self.message_id = message_id
        try:
            self.error_code = ErrorCode(error_code)
            name = self.error_code.name
        except ValueError:
            self.error_code = error_code
            name = "UNKNOWN_ERROR"
        description = ERROR_DESCRIPTIONS.get(error_code, "Unknown controller error")
        super().__init__(f"{name} ({error_code}) for command 0x{command:02X}, "
                         f"message {message_id}: {description}")
