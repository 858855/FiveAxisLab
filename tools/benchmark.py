"""Create synthetic benchmark fixtures and run the same packaged/core CLI."""
import argparse
import concurrent.futures
import json
import math
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
EXAMPLES=ROOT/'examples'

def obj(path,vertices,faces):
    path.write_text(''.join('v %.9g %.9g %.9g\n'%tuple(v) for v in vertices)+
                    ''.join('f %d %d %d\n'%tuple(i+1 for i in f) for f in faces),encoding='utf-8')

def fixtures():
    geometry=EXAMPLES/'geometry';geometry.mkdir(parents=True,exist_ok=True)
    vertices=[(-2.5,-2.5,0),(2.5,-2.5,0),(2.5,2.5,0),(-2.5,2.5,0),
              (-2.5,-2.5,8),(2.5,-2.5,8),(2.5,2.5,8),(-2.5,2.5,8)]
    faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),
           (2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    obj(geometry/'stress-block.obj',vertices,faces)
    cells=[(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6),(0,5,1,6)]
    data='''<VTKFile type="UnstructuredGrid" version="0.1" byte_order="LittleEndian"><UnstructuredGrid><Piece NumberOfPoints="8" NumberOfCells="6">
<Points><DataArray type="Float64" NumberOfComponents="3" format="ascii">%s</DataArray></Points>
<Cells><DataArray type="Int32" Name="connectivity" format="ascii">%s</DataArray><DataArray type="Int32" Name="offsets" format="ascii">4 8 12 16 20 24</DataArray><DataArray type="UInt8" Name="types" format="ascii">10 10 10 10 10 10</DataArray></Cells>
<PointData><DataArray type="Float64" Name="stress" NumberOfComponents="6" format="ascii">%s</DataArray></PointData></Piece></UnstructuredGrid></VTKFile>'''
    (geometry/'stress-tensor.vtu').write_text(data%(' '.join(str(x) for v in vertices for x in v),
            ' '.join(str(x) for c in cells for x in c),' '.join(['11 1 11 0 0 10']*8)),encoding='utf-8')
    n,m=24,12;torus=[];tri=[]
    for i in range(n):
        u=2*math.pi*i/n
        for j in range(m):
            v=2*math.pi*j/m;r=5+1.5*math.cos(v)
            torus.append((r*math.cos(u),r*math.sin(u),1.5+1.5*math.sin(v)))
    for i in range(n):
        for j in range(m):
            a=i*m+j;b=((i+1)%n)*m+j;c=((i+1)%n)*m+(j+1)%m;d=i*m+(j+1)%m
            tri.extend([(a,b,c),(a,c,d)])
    obj(geometry/'torus.obj',torus,tri)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cli',default=str(ROOT/'build/bin/fiveaxis_cli.exe'))
    parser.add_argument('--generate-only',action='store_true');parser.add_argument('--case',action='append');args=parser.parse_args();fixtures()
    if args.generate_only:return
    env=os.environ.copy();env['PATH']=str(ROOT/'sdk/ucrt64/bin')+os.pathsep+env['PATH']
    reportDir=EXAMPLES/'reports';reportDir.mkdir(exist_ok=True)
    cases=[('reference-klipper',['--validate',str(EXAMPLES/'reference-bed.5axis'),'--klipper','--replay',
                               '--gcode',str(EXAMPLES/'reference-klipper.gcode'),'--profile',str(EXAMPLES/'reference-klipper.5machine')]),
           ('reference-head',['--demo',str(EXAMPLES/'reference-head.5axis'),'--head','--replay',
                              '--profile',str(EXAMPLES/'reference-head.5machine')]),
           ('curved-stress',['--slice',str(EXAMPLES/'geometry/stress-block.obj'),'--fem',str(EXAMPLES/'geometry/stress-tensor.vtu'),
                             '--stress-array','stress','--output',str(EXAMPLES/'curved-stress.5axis')]),
           ('torus',['--slice',str(EXAMPLES/'geometry/torus.obj'),'--output',str(EXAMPLES/'torus.5axis')])]
    def run(case):
        name,options=case
        start=time.monotonic();log=reportDir/(name+'.log')
        with log.open('w',encoding='utf-8') as output:
            try:
                p=subprocess.run([args.cli,*options,'--report',str(reportDir/(name+'.json'))],cwd=ROOT,env=env,stdout=output,stderr=output,timeout=180)
                code=p.returncode
            except subprocess.TimeoutExpired:
                code=124;output.write('\nBenchmark time budget exceeded; geometry is not certified.\n')
        result={'fixture':name,'exitCode':code,'seconds':round(time.monotonic()-start,3)}
        print(json.dumps(result),flush=True);return result
    if args.case:cases=[case for case in cases if case[0] in args.case]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,cases))
    previous=json.loads((reportDir/'benchmarks.json').read_text()) if (reportDir/'benchmarks.json').exists() else []
    byName={item['fixture']:item for item in previous+results}
    (reportDir/'benchmarks.json').write_text(json.dumps(list(byName.values()),indent=2),encoding='utf-8')

if __name__=='__main__':main()
