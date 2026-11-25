"""Tells the build what revision and version it is.

Emits the two defines PlatformIO injects: GIT_REV, always the short commit, and
FIRMWARE_VERSION, which is what the tracker reports to the server. The app
compares that against the version in the manifest published beside the image,
so the two have to agree - otherwise the app offers an update to a tracker that
already has it, forever.
"""

import subprocess
import os

revision = ""

env_rev = os.environ.get("GIT_REV")
if not env_rev is None and env_rev != "":
    revision = env_rev
else:
    try:
        revision = (
            subprocess.check_output(["git", "rev-parse", "--short", "HEAD"])
            .strip()
            .decode("utf-8")
        )
    except Exception:
        revision = "NOT_GIT"

# An explicit version always wins. The release workflow passes it, so the
# firmware reports exactly what the manifest beside it says rather than
# whatever git happens to think at build time.
tag = os.environ.get("FIRMWARE_VERSION", "").strip()

if tag == "":
    try:
        # Decode BEFORE splitting. check_output returns bytes and this split
        # them on a str, which throws - so the tag was thrown away every single
        # time, and a tagged release build still called itself git-<rev>.
        tag = (
            subprocess.check_output(
                [
                    "git",
                    "--no-pager",
                    "tag",
                    "--sort",
                    "-taggerdate",
                    "--points-at",
                    "HEAD",
                ]
            )
            .decode("utf-8")
            .split("\n")[0]
            .strip()
        )
    except Exception:
        tag = ""

# Tags here are tracker-v1.0.0 and dongle-v1.0.0, while the manifest and the
# server both use the bare 1.0.0.
for prefix in ("tracker-v", "dongle-v", "v"):
    if tag.startswith(prefix):
        tag = tag[len(prefix) :]
        break

branch = ""
try:
    branch = (
        subprocess.check_output(["git", "symbolic-ref", "--short", "-q", "HEAD"])
        .strip()
        .decode("utf-8")
    )
except Exception:
    branch = ""

# One define, not two. This used to add a second -DFIRMWARE_VERSION on the else
# branch even when a tag had been found, and the last one wins, so the tag was
# overwritten by git-<rev> anyway.
if tag != "":
    version = tag
elif branch != "":
    version = branch
else:
    version = f"git-{revision}"

print(f"'-DGIT_REV=\"{revision}\"' '-DFIRMWARE_VERSION=\"{version}\"'")
