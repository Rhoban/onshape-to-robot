"""
Export every test case (cases.json) in every format from recorded Onshape API
responses, and compare with the expected outputs in tests/expected/.

No network access, API key or API credit is needed. When a change is meant to
modify the exports, regenerate the expected outputs and review their diff:

    pytest tests --update-expected
"""

import difflib
import math
import re
from pathlib import Path

import pytest

from export_case import FORMATS, export, load_cases
from onshape_replay import replay

EXPECTED_DIR = Path(__file__).parent / "expected"
CASES = load_cases()

# A value rounded to zero can be written "-0" or "0" depending on tiny numerical
# differences between numpy versions: they are considered equal
NEGATIVE_ZERO = re.compile(r'(?<![\w.\-])-0(?=[\s"<])')

# q and -q are the same rotation: when w is zero, tiny numerical differences
# can flip the sign of a quaternion, so they are written with their first
# non-zero component positive
QUATERNION = re.compile(r'quat="([^"]*)"')


def canonical_quaternion(match: re.Match) -> str:
    values = [float(value) for value in match.group(1).split()]
    first_non_zero = next((value for value in values if value != 0), 0)
    if first_non_zero < 0:
        values = [-value for value in values]
    return 'quat="' + " ".join(f"{value + 0:g}" for value in values) + '"'


# An Euler angle of +pi or -pi is the same rotation: near pi, tiny numerical
# differences between machines (e.g. the CPU of the CI) can flip its sign. It
# is written +pi in the URDF rpy attributes and in the angles of SDF poses
RPY = re.compile(r'rpy="([^"]*)"')
POSE = re.compile(r"(<pose[^>]*>)([^<]*)(</pose>)")


def canonical_angles(angles: list) -> list:
    return ["3.14159" if abs(abs(float(a)) - math.pi) < 1e-5 else a for a in angles]


def canonical_rpy(match: re.Match) -> str:
    return 'rpy="' + " ".join(canonical_angles(match.group(1).split())) + '"'


def canonical_pose(match: re.Match) -> str:
    values = match.group(2).split()
    if len(values) == 6:
        values = values[:3] + canonical_angles(values[3:])
    return match.group(1) + " ".join(values) + match.group(3)


def normalize(text: str) -> str:
    text = QUATERNION.sub(canonical_quaternion, NEGATIVE_ZERO.sub("0", text))
    return POSE.sub(canonical_pose, RPY.sub(canonical_rpy, text))


@pytest.mark.parametrize("output_format", list(FORMATS))
@pytest.mark.parametrize("case", CASES, ids=[case["name"] for case in CASES])
def test_export(case, output_format, tmp_path, request):
    expected = EXPECTED_DIR / case["name"] / f"robot.{FORMATS[output_format]}"
    expected_error = expected.parent / f"{expected.name}.error"

    try:
        with replay(case["name"]):
            output = export(case, output_format, tmp_path)
        result, error = output.read_text(encoding="utf-8"), None
    except (Exception, SystemExit) as e:
        result, error = None, f"{type(e).__name__}: {e}\n"

    if request.config.getoption("--update-expected"):
        expected.parent.mkdir(parents=True, exist_ok=True)
        if error is None:
            expected.write_text(result, encoding="utf-8")
            expected_error.unlink(missing_ok=True)
        else:
            expected_error.write_text(error, encoding="utf-8")
            expected.unlink(missing_ok=True)
        return

    if expected_error.exists():
        # The export is expected to fail, with this error
        assert error == expected_error.read_text(encoding="utf-8")
        return

    assert error is None, f"export failed: {error}"
    assert expected.exists(), f"no expected output, run pytest --update-expected"
    wanted = expected.read_text(encoding="utf-8")
    if normalize(result) != normalize(wanted):
        diff = "".join(
            list(
                difflib.unified_diff(
                    wanted.splitlines(True),
                    result.splitlines(True),
                    f"expected/{case['name']}/{expected.name}",
                    "export",
                )
            )[:200]
        )
        pytest.fail(f"export differs from the expected output:\n{diff}")
