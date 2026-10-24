"""Package a built PicoVR Dongle firmware for a GitHub release.

Merges bootloader, partition table, boot_app0 and the app into one image that
flashes at 0x0, and writes picovr-dongle.json next to it with the version,
size and SHA-256 the PicoVR server checks before flashing.

Usage: python scripts/package.py <output dir> [expected version]
Run from dongle-firmware/ after `pio run`. With an expected version (the
release tag without its "dongle-v" prefix) it fails if the source disagrees.
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
BUILD = HERE / '.pio' / 'build' / 'xiao_esp32c3'
IMAGE = 'picovr-dongle-esp32c3.bin'


def source_version():
    text = (HERE / 'src' / 'main.cpp').read_text(encoding='utf-8')
    parts = [re.search(rf'FIRMWARE_{p} = (\d+);', text) for p in ('MAJOR', 'MINOR', 'PATCH')]
    if not all(parts):
        sys.exit('Could not read FIRMWARE_MAJOR/MINOR/PATCH from src/main.cpp')
    return '.'.join(m.group(1) for m in parts)


def link_version():
    text = (HERE.parent / 'src' / 'network' / 'picolink.h').read_text(encoding='utf-8')
    return int(re.search(r'LINK_VERSION = (\d+);', text).group(1))


def packages_dir():
    core = os.environ.get('PLATFORMIO_CORE_DIR', str(Path.home() / '.platformio'))
    return Path(core) / 'packages'


def find_one(pattern):
    hits = sorted(glob.glob(str(packages_dir() / pattern)))
    if not hits:
        sys.exit(f'Not found: {pattern}')
    return hits[-1]


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'dist'
    version = source_version()
    if len(sys.argv) > 2 and sys.argv[2] != version:
        sys.exit(f'Tag says {sys.argv[2]} but src/main.cpp says {version}')

    out_dir.mkdir(parents=True, exist_ok=True)
    image = out_dir / IMAGE
    esptool = find_one('tool-esptoolpy*/esptool.py')
    boot_app0 = find_one('framework-arduinoespressif32*/tools/partitions/boot_app0.bin')
    subprocess.run([
        sys.executable, esptool, '--chip', 'esp32c3', 'merge_bin', '-o', str(image),
        '--flash_mode', 'keep', '--flash_freq', 'keep', '--flash_size', 'keep',
        '0x0', str(BUILD / 'bootloader.bin'),
        '0x8000', str(BUILD / 'partitions.bin'),
        '0xe000', boot_app0,
        '0x10000', str(BUILD / 'firmware.bin'),
    ], check=True)

    data = image.read_bytes()
    manifest = {
        'name': 'PicoVR Dongle',
        'version': version,
        'linkVersion': link_version(),
        'chip': 'esp32c3',
        'board': 'xiao_esp32c3',
        'file': IMAGE,
        'offset': 0,
        'size': len(data),
        'sha256': hashlib.sha256(data).hexdigest(),
    }
    (out_dir / 'picovr-dongle.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
