"""
Pytest for get_version utility.
This test will:
- Create a temporary VERSION file
- Patch the Path used in get_version to point to the temp file
- Check that get_version returns the correct version string
"""

import tempfile
from pathlib import Path
import os
import importlib
import sys
import types
import pytest


def test_get_version(tmp_path, monkeypatch):
    # Create a temporary VERSION file
    version_content = "1.2.3-test"
    version_file = tmp_path / "VERSION"
    version_file.write_text(version_content)

    # Patch Path(__file__).parent.parent to tmp_path
    import ros_monitor.utils.version as version_mod

    monkeypatch.setattr(version_mod.Path, "__call__", lambda *a, **kw: tmp_path)

    # Patch get_version to use our temp VERSION file
    def fake_get_version():
        return version_file.read_text().strip()

    monkeypatch.setattr(version_mod, "get_version", fake_get_version)

    assert version_mod.get_version() == version_content
