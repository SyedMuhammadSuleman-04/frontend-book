"""
Comprehensive Logging Module
Provides logging functionality for the VLA system
"""

import logging
import os
from datetime import datetime
from typing import Any
import json


class VLAFormatter(logging.Formatter):
    """
    Custom formatter for VLA system logs.
    """

    def format(self, record):
        """
        Format a log record with VLA-specific information.

        Args:
            record: Log record to format

        Returns:
            Formatted log string
        """
        # Create a dictionary with log information
        log_data = {
            'timestamp': datetime.fromtimestamp(record.created).isoformat(),
            'level': record.levelname,
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'message': record.getMessage(),
        }

        # Add extra fields if they exist
        if hasattr(record, 'user_id'):
            log_data['user_id'] = record.user_id
        if hasattr(record, 'session_id'):
            log_data['session_id'] = record.session_id
        if hasattr(record, 'task_id'):
            log_data['task_id'] = record.task_id

        # Convert to JSON string
        return json.dumps(log_data)


class VLAConsoleHandler(logging.StreamHandler):
    """
    Console handler with color coding for different log levels.
    """
    COLORS = {
        'DEBUG': '\033[36m',    # Cyan
        'INFO': '\033[32m',     # Green
        'WARNING': '\033[33m',  # Yellow
        'ERROR': '\033[31m',    # Red
        'CRITICAL': '\033[35m', # Magenta
        'RESET': '\033[0m'      # Reset
    }

    def emit(self, record):
        """
        Emit a log record to console with color coding.

        Args:
            record: Log record to emit
        """
        try:
            msg = self.format(record)
            color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
            reset = self.COLORS['RESET']
            formatted_msg = f"{color}{msg}{reset}"
            stream = self.stream
            stream.write(formatted_msg + self.terminator)
            self.flush()
        except Exception:
            self.handleError(record)


class VLAFileHandler(logging.FileHandler):
    """
    File handler for VLA system logs.
    """
    def __init__(self, filename: str, mode='a', encoding=None, delay=False):
        """
        Initialize the file handler with automatic directory creation.

        Args:
            filename: Path to the log file
            mode: File mode
            encoding: File encoding
            delay: Whether to delay file opening
        """
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(filename)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        super().__init__(filename, mode, encoding, delay)


class VLALogger:
    """
    Main logger class for the VLA system.
    """

    def __init__(self, name: str = 'VLA', log_file: str = None, level: int = logging.INFO):
        """
        Initialize the VLA logger.

        Args:
            name: Logger name
            log_file: Path to log file (optional)
            level: Logging level
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Prevent adding handlers multiple times
        if not self.logger.handlers:
            # Create formatter
            formatter = VLAFormatter()

            # Add console handler
            console_handler = VLAConsoleHandler()
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

            # Add file handler if specified
            if log_file:
                file_handler = VLAFileHandler(log_file)
                file_handler.setFormatter(formatter)
                self.logger.addHandler(file_handler)

        # Set the logger level
        self.logger.setLevel(level)

    def debug(self, message: str, **kwargs):
        """
        Log a debug message.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.debug(message, extra=extra)

    def info(self, message: str, **kwargs):
        """
        Log an info message.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.info(message, extra=extra)

    def warning(self, message: str, **kwargs):
        """
        Log a warning message.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.warning(message, extra=extra)

    def error(self, message: str, **kwargs):
        """
        Log an error message.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.error(message, extra=extra)

    def critical(self, message: str, **kwargs):
        """
        Log a critical message.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.critical(message, extra=extra)

    def exception(self, message: str, **kwargs):
        """
        Log an exception with traceback.

        Args:
            message: Log message
            **kwargs: Additional fields to include in the log
        """
        extra = kwargs if kwargs else None
        self.logger.exception(message, extra=extra)


# Global logger instance
vla_logger = VLALogger(
    name='VLA',
    log_file='logs/vla_system.log',
    level=logging.INFO
)


def get_logger() -> VLALogger:
    """
    Get the global VLA logger instance.

    Returns:
        VLALogger instance
    """
    return vla_logger


# Example usage function
def example_usage():
    """Example of how to use the VLALogger."""
    print("VLA Logger example usage:")
    print("-" * 25)

    # Get the global logger instance
    logger = get_logger()

    # Log different types of messages
    logger.info("VLA System initialized", component="main", version="0.0.1")
    logger.debug("Processing voice command", task_id="task_123", user_id="user_456")
    logger.warning("Low confidence in speech recognition", confidence=0.65)
    logger.error("Failed to execute action", action_type="navigate", error_code="NAV_001")

    print("\nCheck the logs/vla_system.log file for the log entries.")


if __name__ == "__main__":
    example_usage()