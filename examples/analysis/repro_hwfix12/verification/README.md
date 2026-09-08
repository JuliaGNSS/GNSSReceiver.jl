Verified on orin2 in a separate checkout directory on 2026-09-08:

- `setup.sh` fetched the exact dependency revisions and instantiated successfully.
- The manifest SHA256 remained `f926fbeadd8475b50b9ef0914623b2f89aee0899e3e7853973c22fcf1fc1e1ce` before and after instantiation.
- Loading all four packages succeeded; `load.log` records their versions and resolved source paths.
- `run.sh` was checked with a stand-in Julia executable: source/helper paths, arguments, thread configuration, and environment defaults matched the documented command. This packaging check did not start another RF run.
- Both evidence archives passed `gzip -t`; their SHA256 values are in `../archive.txt`.

The full hardware and regression-test results precede this packaging-only check
and are recorded in `../../hardware_fix_12.md`.
