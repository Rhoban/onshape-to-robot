import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))


def pytest_addoption(parser):
    parser.addoption(
        "--update-expected",
        action="store_true",
        help="Write the exports as the new expected outputs instead of comparing",
    )
