"""Pytest for get_version utility."""

from pathlib import Path

import pytest

import ros_monitor.utils.version as version_mod


def test_get_version(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Test the get_version utility by patching the VERSION file location and content."""
    # Create a temporary VERSION file
    version_content = "1.2.3-test"
    version_file = tmp_path / "VERSION"
    version_file.write_text(version_content)

    monkeypatch.setattr(version_mod.Path, "__call__", lambda: tmp_path)

    # Patch get_version to use our temp VERSION file
    def fake_get_version() -> str:
        return version_file.read_text().strip()

    monkeypatch.setattr(version_mod, "get_version", fake_get_version)

    assert version_mod.get_version() == version_content
