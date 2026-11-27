# Regression tests

These tests export real robots and compare the result with expected outputs,
**without network access, API key or API credit**: the Onshape API responses
are recorded once and replayed.

```bash
pip install -e . pytest
# Optional, for the tests using them
pip install mujoco coacd trimesh
pytest tests
```

## What is tested

`tests/cases.json` lists the test cases. Each one is an assembly of a public
Onshape document from the Rhoban "onshape-to-robot tests" folder, pinned to
its `tests-v1` version (a frozen snapshot of the document), so the cases never
change, even if the documents are edited. Every case is exported to URDF, SDF
and MuJoCo (`tests/test_export.py`) and compared with `tests/expected/<case>/`.
An export that is expected to fail is described by a `robot.<ext>.error` file.

The cases cover real robots (quadrupeds, humanoids, arms, wheeled robots,
parallel robots with closed loops...) and synthetic cases for degrees of
freedom inside sub-assemblies.

The other tests use the same recorded robots:

- `tests/test_models.py` checks that the exports are valid models, not only
  identical to the expected outputs: MuJoCo loads and simulates every case,
  URDF and SDF are well-formed trees, and the three formats describe the same
  joints, limits and masses;
- `tests/test_options.py` checks the effect of the `config.json` entries and
  of the command line options (`--retrieve`, `--convert`, `--safe`);
- `tests/test_regressions.py` contains a test for each fixed bug, reproducing
  its situation (on a robot built in Python, or on a recorded robot).

Some tests need an external tool and are skipped when it is not installed:
MuJoCo (`pip install mujoco`), CoACD (`pip install coacd trimesh`), OpenSCAD
and Gazebo (`gz`, installed in a separate job of the GitHub workflow).
`tests/helpers.py` contains the helpers shared by the tests.

## How it works

`tests/onshape_replay.py` replaces `Onshape.request` (and disables the
onshape-to-robot cache) for the duration of a test. Every request is looked up
by method, path and query string in the case's cassette:

- `tests/data/cassettes/<case>.json.gz`: JSON answers, and references to binary
  ones;
- `tests/data/blobs/<sha256>.gz`: binary answers (STL meshes), shared by cases.

A request with no recorded answer makes the test fail with `MissingRecording`.

## When a change modifies the exports

Regenerate the expected outputs and review their diff, which is part of the
pull request:

```bash
pytest tests --update-expected
git diff tests/expected
```

## Adding a case or recording new requests

1. Add the case to `tests/cases.json` (use a version, `/v/...`, never a
   workspace, so that the case is frozen).
2. List the requests missing from the cassettes. This doesn't use the network:
   requests not recorded yet get placeholder answers, so that the export goes
   on and reveals the next ones.
   ```bash
   python tests/record.py missing <case>
   ```
3. Record them, then repeat 2 and 3 until nothing is missing (a few rounds are
   needed: the assembly reveals the parts, which reveal their meshes...):
   ```bash
   # With an API key (ONSHAPE_API, ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY);
   # it asks for confirmation and costs one API call per request
   python tests/record.py api <case>
   # Or import answers fetched another way, as <case>__<n>.json files containing
   # {"key", "status", "json"}, or {"key", "status", "bin": "<file>"} with the
   # raw body in <file>
   python tests/record.py import <directory>
   ```
4. `pytest tests --update-expected` to write its expected outputs.
