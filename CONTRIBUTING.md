# Contributing

- Each check lives in `Doctor.diagnose()` in `nav2_doctor/doctor.py` and is pure logic: add a test in `test/test_doctor.py` with an injected clock, no ROS needed.
- Run `python3 -m pytest -q test` before opening a PR.
- Bump the version in `doctor.py`, `package.xml` and `setup.py` together (a test enforces it) and add a `CHANGELOG.md` entry.
- Keep commit messages short, one line.
