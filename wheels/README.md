This directory is intended to hold prebuilt wheels for zenoh-python (and any
other binary wheels needed for provisioning constrained devices).

Usage
-----
- Place the wheel file(s) here (for example: `zenoh_python-0.11.0-cp311-cp311-manylinux2014_aarch64.whl`).
- Alternatively, run the helper script at `tools/add_wheel.sh` from the repo root:

  ```bash
  ./tools/add_wheel.sh /path/to/zenoh_python-...whl
  ```

Notes
-----
- Large binary files ( > ~50MB ) should usually be managed with Git LFS. If a wheel is larger than
  GitHub's hard file size limit (~100MB), pushing will fail; use Git LFS in that case:

  ```bash
  git lfs install
  git lfs track 'wheels/*.whl'
  git add .gitattributes
  git commit -m "Track wheels with Git LFS"
  ```

- The repository's provisioning already looks for wheels under `/opt/psyched/wheels`
  (i.e., `REPO_DIR/wheels`). When a wheel matching the host arch and Python tag is
  present, provisioning will install it instead of building locally.

Security
--------
- Only add wheels you trust (signed by you or a trusted maintainer). Installing arbitrary wheels
  can run native code on the device.
