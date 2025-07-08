"""Logging configuration for the ROS2 monitor application."""

import logging
from datetime import UTC, datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = REPO_ROOT / "logs"
LOG_DIR.mkdir(exist_ok=True)

log_filename = LOG_DIR / f"ros2monitor_{datetime.now(UTC).strftime('%Y%m%d')}.log"

logging.basicConfig(
    filename=str(log_filename), level=logging.INFO, format="%(asctime)s [%(levelname)s] %(funcName)s: %(message)s",
)
logger = logging.getLogger("ros2monitor")
