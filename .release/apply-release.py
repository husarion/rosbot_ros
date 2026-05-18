#!/usr/bin/env python3
"""Apply a release: bump <version> in every package.xml + prepend a CHANGELOG section.

Usage:
  apply-release.py <full_tag> <section_file>

  full_tag      e.g. 1.0.1 or 1.0.1-jazzy-mavlink (no leading 'v' — rosbot_ros
                tag convention is bare semver, optionally with a track suffix).
  section_file  path to a file containing the Keep-a-Changelog body for
                this release (### Added / Changed / Fixed / Removed
                blocks). No top-level header — this script prepends it.

Writes:
  CHANGELOG.md          — new "## [<tag>] - YYYY-MM-DD" section at the top.
  */package.xml         — each <version>...</version> bumped to the X.Y.Z
                          part of the tag (the track suffix doesn't go into
                          package.xml because rosdep / ament don't grok it).

Run from the repo root. Bumps every package.xml in the tree in lockstep —
this repo treats the workspace as one releasable unit.
"""

import datetime as dt
import re
import shutil
import sys
from pathlib import Path

CHANGELOG = Path("CHANGELOG.md")
VERSION_TAG = re.compile(r"^(\d+\.\d+\.\d+)(?:-[A-Za-z0-9.\-]+)?$")


def prepend_section(tag: str, section_body: str) -> None:
    if not CHANGELOG.is_file():
        sys.exit(f"{CHANGELOG} missing — bootstrap it before releasing.")
    today = dt.date.today().isoformat()
    new_section = f"## [{tag}] - {today}\n\n{section_body.rstrip()}\n\n"

    text = CHANGELOG.read_text()
    m = re.search(r"(?m)^## \[", text)
    if m:
        new_text = text[: m.start()] + new_section + text[m.start() :]
    else:
        if not text.endswith("\n\n"):
            text = text.rstrip("\n") + "\n\n"
        new_text = text + new_section

    shutil.copy(CHANGELOG, CHANGELOG.with_suffix(CHANGELOG.suffix + ".bak"))
    CHANGELOG.write_text(new_text)
    print(f"prepended CHANGELOG.md section: ## [{tag}] - {today}")


def bump_package_xmls(version_xyz: str) -> int:
    pattern = re.compile(r"(<version>)[^<]+(</version>)")
    pkgs = sorted(
        p
        for p in Path(".").rglob("package.xml")
        if "build/" not in str(p) and "install/" not in str(p)
    )
    if not pkgs:
        sys.exit("no package.xml files found under cwd")
    for pkg in pkgs:
        text = pkg.read_text()
        new_text, n = pattern.subn(rf"\g<1>{version_xyz}\g<2>", text, count=1)
        if n == 0:
            sys.exit(f"{pkg}: no <version> tag found")
        shutil.copy(pkg, pkg.with_suffix(pkg.suffix + ".bak"))
        pkg.write_text(new_text)
        print(f"bumped {pkg} -> <version>{version_xyz}</version>")
    return len(pkgs)


def main() -> None:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    tag, section_file = sys.argv[1], Path(sys.argv[2])
    m = VERSION_TAG.match(tag)
    if not m:
        sys.exit(f"tag must look like X.Y.Z[-<suffix>], got: {tag}")
    version_xyz = m.group(1)
    if not section_file.is_file():
        sys.exit(f"section file not found: {section_file}")
    body = section_file.read_text()
    if not body.strip():
        sys.exit("section file is empty")

    prepend_section(tag, body)
    n = bump_package_xmls(version_xyz)
    print(f"updated {n} package.xml files to {version_xyz}")


if __name__ == "__main__":
    main()
