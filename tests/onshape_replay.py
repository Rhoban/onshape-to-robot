"""
Replay recorded Onshape API responses, so that exports can be tested without
network access, API keys or API credits.

Responses are recorded at the HTTP level (Onshape.request), keyed by method,
path and query string, in one cassette per test case:

    tests/data/cassettes/<case>.json.gz   {key: {"status": int, "json": ...}}
                                          {key: {"status": int, "blob": sha256}}
    tests/data/blobs/<sha256>.gz          binary bodies (e.g. STL), shared by cases
"""

from __future__ import annotations

import contextlib
import gzip
import hashlib
import json
import sys
import tempfile
import urllib.parse
from pathlib import Path

from onshape_to_robot.onshape_api import cache as cache_module
from onshape_to_robot.onshape_api import onshape as onshape_module

DATA_DIR = Path(__file__).parent / "data"
CASSETTES_DIR = DATA_DIR / "cassettes"
BLOBS_DIR = DATA_DIR / "blobs"

EMPTY_STL = b"\0" * 80 + b"\0\0\0\0"


def request_key(method: str, path: str, query: dict) -> str:
    """Same URL encoding as Onshape.request"""
    return f"{method.upper()} {path}?{urllib.parse.urlencode(query)}"


class MissingRecording(Exception):
    pass


class FakeResponse:
    def __init__(self, status_code: int, content: bytes):
        self.status_code = status_code
        self.content = content
        self.headers = {}

    @property
    def text(self) -> str:
        return self.content.decode("utf-8", errors="replace")

    def json(self):
        return json.loads(self.content)


class Cassette:
    def __init__(self, case: str):
        self.case = case
        self.path = CASSETTES_DIR / f"{case}.json.gz"
        self.entries: dict = {}
        if self.path.exists():
            with gzip.open(self.path, "rt", encoding="utf-8") as f:
                self.entries = json.load(f)

    def get(self, key: str) -> FakeResponse | None:
        entry = self.entries.get(key)
        if entry is None:
            return None
        if "blob" in entry:
            with gzip.open(BLOBS_DIR / f"{entry['blob']}.gz", "rb") as f:
                content = f.read()
        else:
            content = json.dumps(entry["json"]).encode("utf-8")
        return FakeResponse(entry["status"], content)

    def add(self, key: str, status: int, content: bytes, is_json: bool):
        if is_json:
            self.entries[key] = {"status": status, "json": json.loads(content)}
        else:
            sha = hashlib.sha256(content).hexdigest()
            BLOBS_DIR.mkdir(parents=True, exist_ok=True)
            blob = BLOBS_DIR / f"{sha}.gz"
            if not blob.exists():
                with gzip.GzipFile(blob, "wb", mtime=0) as f:
                    f.write(content)
            self.entries[key] = {"status": status, "blob": sha}

    def save(self):
        CASSETTES_DIR.mkdir(parents=True, exist_ok=True)
        with gzip.GzipFile(self.path, "wb", mtime=0) as f:
            f.write(
                json.dumps(self.entries, sort_keys=True, separators=(",", ":")).encode()
            )


def _stub(path: str) -> bytes | None:
    """Placeholder answers used while collecting the requests of a case"""
    if path.endswith("/stl"):
        return EMPTY_STL
    for suffix, value in [
        ("/massproperties", {"bodies": {}}),
        ("/variables", []),
        ("/matevalues", {"mateValues": []}),
        ("/features", {"features": []}),
        ("/configuration", {"configurationParameters": []}),
    ]:
        if path.endswith(suffix):
            return json.dumps(value).encode()
    if path.startswith("/api/metadata/"):
        return json.dumps({"properties": []}).encode()
    return None


@contextlib.contextmanager
def replay(case: str, missing: list | None = None):
    """
    Serve Onshape requests from the case's cassette.

    If missing is None, an unrecorded request raises MissingRecording.
    Otherwise (collect mode) unrecorded request keys are appended to missing,
    and a placeholder answer is returned when possible so that the export can
    go on and reveal the next requests.
    """
    cassette = Cassette(case)
    original_init = onshape_module.Onshape.__init__
    original_request = onshape_module.Onshape.request
    original_cache_path = cache_module.get_cache_path

    def fake_init(self, stack, creds="./config.json", logging=True):
        self._logging = False
        self._url = "https://cad.onshape.com"
        self._access_key = self._secret_key = b"replay"

    def fake_request(
        self, method, path, query={}, headers={}, body={}, base_url=None
    ):
        key = request_key(method, path, query)
        response = cassette.get(key)
        if response is None:
            if missing is None:
                raise MissingRecording(f"[{case}] no recorded response for {key}")
            if key not in missing:
                missing.append(key)
            stub = _stub(path)
            if stub is None:
                raise MissingRecording(f"[{case}] no recorded response for {key}")
            response = FakeResponse(200, stub)
        if not 200 <= response.status_code <= 206:
            # Same behaviour as Onshape.request on an API error
            print(f"! ERROR ({response.status_code}) while using Onshape API: {key}")
            sys.exit(1)
        return response

    with tempfile.TemporaryDirectory() as cache_dir:
        onshape_module.Onshape.__init__ = fake_init
        onshape_module.Onshape.request = fake_request
        cache_module.get_cache_path = lambda: Path(cache_dir)
        try:
            yield cassette
        finally:
            onshape_module.Onshape.__init__ = original_init
            onshape_module.Onshape.request = original_request
            cache_module.get_cache_path = original_cache_path
