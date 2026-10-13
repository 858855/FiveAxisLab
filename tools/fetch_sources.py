"""Fetch fixed upstream releases and an exact OrcaSlicer reference revision."""
import hashlib
import io
import json
from pathlib import Path
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / 'vendor'

def get(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'FiveAxisLab-build'})
    with urllib.request.urlopen(req, timeout=120) as response:
        return response.read()

def main():
    VENDOR.mkdir(exist_ok=True)
    lock = ROOT / 'upstream.lock.json'
    previous = json.loads(lock.read_text(encoding='utf-8')) if lock.exists() else {}
    manifest = {}
    for name, repo, tag in [('libccd', 'danfis/libccd', 'v2.1'), ('fcl', 'flexible-collision-library/fcl', '0.7.0'), ('ompl', 'ompl/ompl', '1.7.0')]:
        destination = VENDOR / name
        if destination.exists():
            continue
        data = get(f'https://codeload.github.com/{repo}/tar.gz/refs/tags/{tag}')
        if name in previous and hashlib.sha256(data).hexdigest()!=previous[name]['sha256']:
            raise RuntimeError('Upstream archive checksum mismatch: '+name)
        destination.mkdir()
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
            for member in archive:
                relative = Path(*Path(member.name).parts[1:])
                if not relative.parts:
                    continue
                target = (destination / relative).resolve()
                if not target.is_relative_to(destination.resolve()) or member.issym() or member.islnk():
                    raise RuntimeError('Unsafe source archive path')
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(archive.extractfile(member).read())
        manifest[name] = {'repository': repo, 'tag': tag, 'sha256': hashlib.sha256(data).hexdigest()}
        print('Fetched', name, tag, flush=True)
    orca = VENDOR / 'orca-reference'
    orca.mkdir(exist_ok=True)
    manifest = {**previous, **manifest}
    sha = previous.get('orca', {}).get('commit')
    if not sha:
        sha = json.loads(get('https://api.github.com/repos/OrcaSlicer/OrcaSlicer/commits/main'))['sha']
    for filename in ['LICENSE.txt', 'AGENTS.md', 'src/libslic3r/Flow.cpp', 'src/libslic3r/Flow.hpp']:
        target = orca / filename
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(get(f'https://raw.githubusercontent.com/OrcaSlicer/OrcaSlicer/{sha}/{filename}'))
    manifest['orca'] = {'repository': 'OrcaSlicer/OrcaSlicer', 'commit': sha}
    klipper = VENDOR / 'klipper'
    if not klipper.exists():
        commit = previous.get('klipper',{}).get('commit') or json.loads(get('https://api.github.com/repos/Klipper3d/klipper/commits/master'))['sha']
        data = get(f'https://codeload.github.com/Klipper3d/klipper/tar.gz/{commit}')
        if 'klipper' in previous and hashlib.sha256(data).hexdigest()!=previous['klipper']['sha256']:
            raise RuntimeError('Klipper archive checksum mismatch')
        klipper.mkdir()
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
            for member in archive:
                relative = Path(*Path(member.name).parts[1:])
                if not relative.parts or member.issym() or member.islnk():
                    continue
                target = (klipper / relative).resolve()
                if not target.is_relative_to(klipper.resolve()):
                    raise RuntimeError('Unsafe Klipper source path')
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(archive.extractfile(member).read())
        manifest['klipper'] = {'repository': 'Klipper3d/klipper', 'commit': commit, 'sha256': hashlib.sha256(data).hexdigest()}
        print('Pinned Klipper', commit, flush=True)
    lock.write_text(json.dumps(manifest, indent=2))
    (ROOT / 'LICENSE').write_bytes((orca / 'LICENSE.txt').read_bytes())
    print('Pinned OrcaSlicer', sha, flush=True)

if __name__ == '__main__':
    main()
