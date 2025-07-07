"""Pytest for logging_config.py."""
import time
import re
from pathlib import Path
from ros_monitor.utils.logging_config import logger, log_filename

def test_logging_config(tmp_path):
    test_message = "pytest logging test message"
    logger.info(test_message)
    # Wait a moment to ensure the log is written
    time.sleep(0.2)
    log_path = Path(log_filename)
    assert log_path.exists(), f"Log file {log_path} does not exist!"
    with open(log_path, "r") as f:
        log_contents = f.read()
    assert re.search(re.escape(test_message), log_contents), "Test message not found in log file!"
