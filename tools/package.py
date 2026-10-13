"""Assemble a portable Windows app by resolving PE imports, then archive its source."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT / 'sdk/ucrt64'
OUT = ROOT / 'dist/FiveAxisLab-0.4.0-win64'

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    candidates = {p.name.lower(): p for directory in [SDK/'bin', ROOT/'build/bin'] for p in directory.glob('*.dll')}
    system = Path(os.environ['SystemRoot'])/'System32'
    queue=[]
    for filename in ['FiveAxisLab.exe', 'fiveaxis_cli.exe']:
        target=OUT/filename
        shutil.copy2(ROOT/'build/bin'/filename,target);queue.append(target)
    for source in (ROOT/'build/bin/compute').glob('*.dll'):
        target=OUT/'compute'/source.name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target);queue.append(target)
    pluginRoot=SDK/'share/qt6/plugins'
    for group in ['platforms','imageformats','styles']:
        for source in (pluginRoot/group).glob('*.dll'):
            if group=='platforms' and source.name!='qwindows.dll':continue
            target=OUT/'plugins'/group/source.name;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,target);queue.append(target)
    # The platform plugin's software OpenGL fallback is loaded dynamically.
    fallback=SDK/'bin/opengl32sw.dll'
    if fallback.exists():shutil.copy2(fallback,OUT/fallback.name);queue.append(OUT/fallback.name)
    checked=set()
    while queue:
        binary=queue.pop();key=binary.name.lower()
        if key in checked:continue
        checked.add(key)
        headers=subprocess.check_output([str(SDK/'bin/objdump.exe'),'-p',str(binary)],text=True,encoding='utf-8',errors='replace')
        for dependency in re.findall(r'DLL Name:\s*([^\s]+)',headers):
            name=dependency.lower()
            if name in checked:continue
            source=candidates.get(name)
            if source:
                target=OUT/source.name;shutil.copy2(source,target);queue.append(target)
            elif not name.startswith(('api-ms-','ext-ms-')) and not (system/dependency).exists():
                raise RuntimeError(f'Missing runtime dependency {dependency} required by {binary.name}')
    (OUT/'qt.conf').write_text('[Paths]\nPlugins=plugins\n',encoding='utf-8')
    for filename in ['README.md','LICENSE','THIRD_PARTY.md','upstream.lock.json','dependencies.lock.json']:
        shutil.copy2(ROOT/filename,OUT/filename)
    for directory in ['controllers','examples','docs']:
        shutil.copytree(ROOT/directory,OUT/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(SDK/'share/licenses',OUT/'licenses/msys2',dirs_exist_ok=True)
    for name in ['libccd','fcl','ompl','orca-reference','klipper']:
        for filename in ['LICENSE','LICENSE.txt','COPYING','LICENSES','BSD-LICENSE']:
            source=ROOT/'vendor'/name/filename
            if source.is_file():
                target=OUT/'licenses'/name/filename;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes={str(p.relative_to(OUT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.rglob('*') if p.is_file() and 'cache' not in p.parts and p.name!='manifest.sha256.json'}
    (OUT/'manifest.sha256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    archive=Path(str(OUT)+'.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in OUT.rglob('*'):
            if p.is_file() and 'cache' not in p.parts:z.write(p, str(p.relative_to(OUT.parent)))
    sourceArchive=ROOT/'dist/FiveAxisLab-0.4.0-source.zip'
    with zipfile.ZipFile(sourceArchive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for directory in ['src','tests','tools','controllers','docs','examples','vendor']:
            for p in (ROOT/directory).rglob('*'):
                if p.is_file() and not any(x in p.parts for x in ['__pycache__','.git','out']) and p.suffix not in ['.dll','.so','.o'] and (p.suffix!='.obj' or directory=='examples'):
                    z.write(p,'FiveAxisLab/'+str(p.relative_to(ROOT)))
        for filename in ['CMakeLists.txt','LICENSE','README.md','THIRD_PARTY.md','dependencies.lock.json','upstream.lock.json','.gitignore','.gitattributes','.clang-format']:
            z.write(ROOT/filename,'FiveAxisLab/'+filename)
    print(json.dumps({'runtimeFiles':len(hashes),'portableZip':str(archive),'sourceZip':str(sourceArchive)},indent=2))

if __name__=='__main__':main()
