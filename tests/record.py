"""
Record the Onshape API responses needed by the test cases.

    # 1. List the requests missing from the cassettes (no network, no API call)
    python tests/record.py missing [case ...]

    # 2a. Fetch them with your API key (costs one API call per request listed)
    python tests/record.py api [case ...]

    # 2b. Or import responses fetched another way (e.g. from a logged-in browser
    #     session), see tests/README.md for the expected files
    python tests/record.py import <directory>

Repeat until "missing" reports nothing: placeholder answers are used for the
requests not recorded yet, so a few rounds can be needed for the export to
reveal every request. Then regenerate the expected outputs:

    pytest tests --update-expected
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import urllib.parse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from export_case import FORMATS, export, load_cases  # noqa: E402
from onshape_replay import DATA_DIR, Cassette, MissingRecording, replay  # noqa: E402

MISSING_DIR = DATA_DIR / "missing"


def selected_cases(names: list[str]) -> list[dict]:
    cases = load_cases()
    if not names:
        return cases
    unknown = set(names) - {case["name"] for case in cases}
    if unknown:
        sys.exit(f"Unknown cases: {', '.join(sorted(unknown))}")
    return [case for case in cases if case["name"] in names]


def collect_missing(case: dict) -> list[str]:
    missing: list[str] = []
    for output_format in FORMATS:
        with tempfile.TemporaryDirectory() as directory:
            try:
                with replay(case["name"], missing=missing):
                    export(case, output_format, Path(directory))
            except (MissingRecording, SystemExit, Exception) as e:
                if not missing:
                    print(f"  [{case['name']}/{output_format}] export failed: {e}")
    return missing


def cmd_missing(args):
    MISSING_DIR.mkdir(parents=True, exist_ok=True)
    total = 0
    for case in selected_cases(args.cases):
        missing = collect_missing(case)
        path = MISSING_DIR / f"{case['name']}.json"
        if missing:
            path.write_text(json.dumps(missing, indent=1))
        elif path.exists():
            path.unlink()
        total += len(missing)
        print(f"{case['name']}: {len(missing)} missing request(s)")
    print(f"Total: {total}")


def cmd_api(args):
    from onshape_to_robot.onshape_api.onshape import Onshape

    todo = []
    for case in selected_cases(args.cases):
        path = MISSING_DIR / f"{case['name']}.json"
        if path.exists():
            todo += [(case["name"], key) for key in json.loads(path.read_text())]
    if not todo:
        print("Nothing to record, run 'missing' first")
        return
    # A request needed by several cases is only made once
    unique_keys = list(dict.fromkeys(key for _, key in todo))
    answer = input(
        f"This will make {len(unique_keys)} Onshape API call(s). Continue? [y/N] "
    )
    if answer.lower() != "y":
        return

    # Keys are read from the ONSHAPE_API / ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY
    # environment variables (an empty credentials file makes Onshape use them)
    empty_creds = Path(tempfile.mkdtemp()) / "creds.json"
    empty_creds.write_text("{}")
    api = Onshape("https://cad.onshape.com", creds=str(empty_creds), logging=False)
    responses = {}
    for index, key in enumerate(unique_keys):
        method, url = key.split(" ", 1)
        path, _, query = url.partition("?")
        query = dict(urllib.parse.parse_qsl(query, keep_blank_values=True))
        response = api.request(method.lower(), path, query=query)
        is_json = "json" in response.headers.get("Content-Type", "")
        responses[key] = (response.status_code, response.content, is_json)
        print(f"  [{index + 1}/{len(unique_keys)}] {response.status_code} {key[:100]}")

    cassettes: dict[str, Cassette] = {}
    for case_name, key in todo:
        cassette = cassettes.setdefault(case_name, Cassette(case_name))
        cassette.add(key, *responses[key])
    for cassette in cassettes.values():
        cassette.save()
    print(f"Recorded {len(unique_keys)} response(s) for {len(cassettes)} case(s)")


def cmd_import(args):
    """
    Import <case>__<n>.json files: {"key", "status", "json"} for JSON answers,
    or {"key", "status", "bin": "<file>"} with the raw body in <file>.
    """
    directory = Path(args.directory)
    cassettes: dict[str, Cassette] = {}
    count = 0
    for meta_file in sorted(directory.glob("*__*.json")):
        case_name = meta_file.name.split("__")[0]
        meta = json.loads(meta_file.read_text())
        cassette = cassettes.setdefault(case_name, Cassette(case_name))
        if "bin" in meta:
            content = (directory / meta["bin"]).read_bytes()
            cassette.add(meta["key"], meta["status"], content, False)
        else:
            cassette.add(meta["key"], meta["status"], json.dumps(meta["json"]).encode(), True)
        count += 1
    for cassette in cassettes.values():
        cassette.save()
    print(f"Imported {count} response(s) into {len(cassettes)} cassette(s)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("missing", "api"):
        p = sub.add_parser(name)
        p.add_argument("cases", nargs="*")
    p = sub.add_parser("import")
    p.add_argument("directory")
    args = parser.parse_args()
    {"missing": cmd_missing, "api": cmd_api, "import": cmd_import}[args.command](args)


if __name__ == "__main__":
    main()
