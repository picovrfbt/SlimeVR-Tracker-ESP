"""Package built PicoVR tracker firmware for a GitHub release.

Each PlatformIO env is merged into one image that flashes at 0x0, and
picovr-tracker.json lists them as variants with the size and SHA-256 the
PicoVR server checks before flashing.

Usage: python scripts/package_tracker.py <output dir> [expected version]
Run from the repo root after building the envs. With an expected version (the
release tag without its "tracker-v" prefix) it only labels the build.
"""
import glob
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent

# Each variant: the PlatformIO env, and what it is, for the app to show.
VARIANTS = [
    {
        'id': 'topaz-lsm6dsv',
        'env': 'picovr_topaz',
        'name': 'PicoVR V3.1 (Topaz, LSM6DSV)',
        'imu': 'LSM6DSV',
        'rotation': 0,
        'default': True,
    },
    {
        'id': 'topaz-bno085',
        'env': 'picovr_topaz_bno',
        'name': 'PicoVR V3.1 (Topaz, BNO085)',
        'imu': 'BNO085',
        'rotation': 180,
        'default': False,
    },
]


def constant(path, pattern, cast=str):
    text = (HERE / path).read_text(encoding='utf-8')
    match = re.search(pattern, text)
    if not match:
        sys.exit(f'Could not find {pattern} in {path}')
    return cast(match.group(1))


def git_version():
    for command in (['git', 'describe', '--tags', '--abbrev=0'], ['git', 'rev-parse', '--short', 'HEAD']):
        try:
            out = subprocess.check_output(command, cwd=HERE, stderr=subprocess.DEVNULL).decode().strip()
            if out:
                return out.removeprefix('tracker-v')
        except Exception:
            continue
    return '0.0.0'


def packages_dir():
    core = os.environ.get('PLATFORMIO_CORE_DIR', str(Path.home() / '.platformio'))
    return Path(core) / 'packages'


def find_one(pattern):
    hits = sorted(glob.glob(str(packages_dir() / pattern)))
    if not hits:
        sys.exit(f'Not found: {pattern}')
    return hits[-1]


def merge(env, image):
    build = HERE / '.pio' / 'build' / env
    if not (build / 'firmware.bin').exists():
        sys.exit(f'{env} has not been built: {build / "firmware.bin"} is missing')
    esptool = find_one('tool-esptoolpy*/esptool.py')
    boot_app0 = find_one('framework-arduinoespressif32*/tools/partitions/boot_app0.bin')
    subprocess.run([
        sys.executable, esptool, '--chip', 'esp32c3', 'merge_bin', '-o', str(image),
        '--flash_mode', 'keep', '--flash_freq', 'keep', '--flash_size', 'keep',
        '0x0', str(build / 'bootloader.bin'),
        '0x8000', str(build / 'partitions.bin'),
        '0xe000', boot_app0,
        '0x10000', str(build / 'firmware.bin'),
    ], check=True)


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'dist-tracker'
    version = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else git_version()
    out_dir.mkdir(parents=True, exist_ok=True)

    built = []
    for variant in VARIANTS:
        name = f'picovr-tracker-{variant["id"]}.bin'
        image = out_dir / name
        merge(variant['env'], image)
        data = image.read_bytes()
        built.append({
            **{k: v for k, v in variant.items() if k != 'env'},
            'chip': 'esp32c3',
            'board': 'PicoVR V3.1',
            'file': name,
            'offset': 0,
            'size': len(data),
            'sha256': hashlib.sha256(data).hexdigest(),
        })

    manifest = {
        'name': 'PicoVR Tracker',
        'version': version,
        'linkVersion': constant('src/network/picolink.h', r'LINK_VERSION = (\d+);', int),
        'protocolVersion': constant('src/debug.h', r'PROTOCOL_VERSION (\d+)', int),
        'variants': built,
    }
    (out_dir / 'picovr-tracker.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
