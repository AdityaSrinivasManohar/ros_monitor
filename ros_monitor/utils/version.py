"""Utility to read the version of the package from a VERSION file."""

from pathlib import Path


def get_version() -> str:
    """Read and return the version string from the VERSION file.

    Returns
    -------
    str
        The version string read from the VERSION file.

    """
    version_file = Path(__file__).parent.parent / "VERSION"
    return version_file.read_text().strip()
