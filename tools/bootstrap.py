"""Install a checksum-verified, workspace-local MSYS2 C++ SDK (no system edits)."""
import concurrent.futures
import hashlib
import io
import json
from pathlib import Path
import re
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.deps-cache'
SDK = ROOT / 'sdk'
BASE = 'https://repo.msys2.org/mingw/ucrt64/'

def download(url, path):
    if not path.exists():
        with urllib.request.urlopen(url, timeout=120) as r:
            path.write_bytes(r.read())
    return path.read_bytes()

def main():
    CACHE.mkdir(exist_ok=True)
    SDK.mkdir(exist_ok=True)
    packages = {}
    lock = ROOT / 'dependencies.lock.json'
    if lock.exists():
        for name, item in json.loads(lock.read_text(encoding='utf-8')).items():
            packages[name]={'VERSION':[item['version']], 'SHA256SUM':[item['sha256']], 'FILENAME':[item['file']]}
    else:
        db = download(BASE + 'ucrt64.db', CACHE / 'ucrt64.db')
        with tarfile.open(fileobj=io.BytesIO(db)) as archive:
            for member in archive:
                if member.name.endswith('/desc'):
                    fields = re.split(r'%([^%]+)%\n', archive.extractfile(member).read().decode())
                    item = {fields[i]: fields[i+1].strip().splitlines() for i in range(1, len(fields), 2)}
                    packages[item['NAME'][0]] = item
    prefix = 'mingw-w64-ucrt-x86_64-'
    roots = ['gcc', 'cmake', 'ninja', 'qt6-base', 'opencascade', 'eigen3', 'boost', 'tetgen', 'libzip', 'cgal']
    wanted = set()
    def visit(name):
        if name in wanted:
            return
        if name not in packages:
            providers = [k for k, v in packages.items() if any(re.split('[<>=]', p)[0] == name for p in v.get('PROVIDES', []))]
            if not providers:
                raise RuntimeError('Missing dependency ' + name)
            name = sorted(providers)[0]
        wanted.add(name)
        for dep in packages[name].get('DEPENDS', []):
            visit(re.split('[<>=]', dep)[0])
    if lock.exists():
        wanted=set(packages)
    else:
        for name in roots:
            visit(prefix + name)
    print('Resolving', len(wanted), 'packages', flush=True)
    def install(name):
        item = packages[name]
        filename = item['FILENAME'][0]
        marker = SDK / '.installed' / filename
        if marker.exists() and marker.read_text().strip()==item['SHA256SUM'][0]:
            return name + ' cached'
        data = download(BASE + filename, CACHE / filename)
        actual = hashlib.sha256(data).hexdigest()
        if actual != item['SHA256SUM'][0]:
            raise RuntimeError('Checksum mismatch: ' + filename)
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:*') as archive:
            links = []
            for member in archive:
                if not member.name.startswith('ucrt64/'):
                    continue
                if member.issym() or member.islnk():
                    links.append(member)
                    continue
                dest = (SDK / member.name).resolve()
                if not dest.is_relative_to(SDK.resolve()):
                    raise RuntimeError('Unsafe archive path')
                if member.isdir():
                    dest.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(archive.extractfile(member).read())
            for member in links:
                dest = (SDK / member.name).resolve()
                source = (SDK / member.linkname if member.islnk() else dest.parent / member.linkname).resolve()
                if not source.is_relative_to(SDK.resolve()) or not dest.is_relative_to(SDK.resolve()):
                    raise RuntimeError('Unsafe archive link')
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(source.read_bytes())
        marker.parent.mkdir(exist_ok=True)
        marker.write_text(actual)
        return name + ' installed'
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        for message in pool.map(install, sorted(wanted)):
            print(message, flush=True)
    (ROOT / 'dependencies.lock.json').write_text(json.dumps({k: {'version': packages[k]['VERSION'][0], 'sha256': packages[k]['SHA256SUM'][0], 'file': packages[k]['FILENAME'][0]} for k in sorted(wanted)}, indent=2))

if __name__ == '__main__':
    main()
