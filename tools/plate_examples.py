"""Generate reproducible synthetic schema-2 plate/feeder/toolchanger fixtures."""
import copy,json,os,subprocess,time,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'examples/plate';OUT.mkdir(parents=True,exist_ok=True)
CLI=ROOT/'build/bin/fiveaxis_cli.exe'
env=os.environ.copy();env['PATH']=str(ROOT/'sdk/ucrt64/bin')+os.pathsep+env['PATH']
env['TEMP']=env['TMP']=str(ROOT/'build/tmp')

def run(name,args):
    start=time.monotonic();log=ROOT/'examples/reports'/f'{name}.log'
    with log.open('w',encoding='utf-8') as output:
        try:
            process=subprocess.run([str(CLI),*map(str,args)],cwd=ROOT,env=env,stdout=output,stderr=output,timeout=300)
            code=process.returncode
        except subprocess.TimeoutExpired:
            code=124;output.write('\nBenchmark time budget exhausted; no complete-job certificate.\n')
    print(name,code,round(time.monotonic()-start,3),flush=True)
    return {'fixture':name,'exitCode':code,'endToEndSeconds':round(time.monotonic()-start,3),'timeBudgetSeconds':300}

def save(name,j):
    file=OUT/(name+'.5axis')
    with zipfile.ZipFile(file,'w',zipfile.ZIP_DEFLATED) as archive:archive.writestr('manifest.json',json.dumps(j))
    return file

def matrix(x,y,z):return [1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1]

def main():
    results=[];base=OUT/'sequential.5axis'
    run('plate-generate',['--plate-demo','--complete-objects','--output',base])
    results.append(run('plate-sequential',['--validate',base,'--output',base,'--threads',4,'--report',ROOT/'examples/reports/plate-sequential.json','--replay']))
    with zipfile.ZipFile(base) as archive:j=json.loads(archive.read('manifest.json'))
    job=j['plateJob'];job['settings']['automaticOrder']=False;j['machine']['controller']=0
    material=copy.deepcopy(job['materials'][0]);material['id']='material-2';material['name']='Second PLA';material['color']='#59bd87';job['materials'].append(material)
    feeder=copy.deepcopy(j);f=feeder['plateJob'];f['objects'][1]['material']='material-2';f['tools'][0]['routes'][0]['command']='M42 P1 S0';f['tools'][0]['routes'].append({'material':'material-2','slot':'1','command':'M42 P1 S1','confirmed':True});f['tools'][0]['change']['load']=5;f['tools'][0]['change']['unload']=5;f['settings']['purge']=[{'from':'material-1','to':'material-2','volume':1},{'from':'material-2','to':'material-1','volume':1}];file=save('shared-nozzle',feeder)
    results.append(run('plate-shared-nozzle',['--validate',file,'--reslice','--threads',4,'--output',file,'--report',ROOT/'examples/reports/plate-shared-nozzle.json','--replay']))
    tools=copy.deepcopy(j);t=tools['plateJob'];second=copy.deepcopy(t['tools'][0]);second.update({'id':'tool-2','name':'Fine dummy tool','number':1,'duetDrive':1,'duetHeater':1,'extruder':'extruder1','heater':'extruder1'});second['nozzle'].update({'diameter':.4,'name':'0.4 mm','minWidth':.32,'maxWidth':.8,'minHeight':.05,'maxHeight':.3,'dummy':True,'mount':matrix(1,0,0)});second['routes']=[{'material':'material-2','slot':'0','command':'','confirmed':True}];t['tools'].append(second);t['objects'][1]['material']='material-2';t['objects'][1]['tool']='tool-2';t['objects'][1]['inheritProcess']=False;t['objects'][1]['data']['process'].update({'nozzleDiameter':.4,'width':.45,'layerHeight':.2});
    for index,tool in enumerate(t['tools']):
        x=-60 if index==0 else 60;tool['detachable']=True;tool['dock']=matrix(x,0,30);change=tool['change'];change['calibrated']=True
        move={'kind':0,'carriage':{'xyz':[x,0,30],'a':0,'b':0},'argument':'','seconds':0,'component':'','fixture':'','center':[0,0,0],'size':[0,0,0]}
        guard={'kind':3,'carriage':move['carriage'],'argument':f'M577 P{index+1} S1','seconds':0,'component':'','fixture':'','center':[0,0,0],'size':[0,0,0]}
        change['pickup']=[move,guard];release=copy.deepcopy(guard);release['argument']=f'M577 P{index+1} S0';change['release']=[move,release]
    carriage=copy.deepcopy(t['tools'][0]['dummy']);carriage['components']=carriage['components'][2:3];carriage['components'][0]['id']='coupler';carriage['components'][0]['name']='Synthetic coupler envelope';carriage['components'][0]['link']=0;carriage['components'][0]['nozzle']=False
    for key in ('mesh','renderMesh'):
        for vertex in carriage['components'][0][key]['vertices']:vertex[2]+=30
    t['carriage']=carriage;file=save('two-tools',tools)
    results.append(run('plate-two-tools',['--validate',file,'--reslice','--threads',4,'--output',file,'--report',ROOT/'examples/reports/plate-two-tools.json','--replay']))
    (ROOT/'examples/reports/plate-benchmarks.json').write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__':main()
