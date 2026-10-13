"""Run real pinned Klippy in Linux debug-file mode and retain explicit evidence."""
import base64
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'build';OUT=ROOT/'examples/reports/klipper'

def run(name,config,gcode,expect_error=None,require_rotary=False):
    OUT.mkdir(parents=True,exist_ok=True)
    cfg=OUT/(name+'.cfg');cfg.write_text(config,encoding='utf-8')
    commands=OUT/(name+'.gcode');commands.write_text(gcode,encoding='utf-8')
    log=OUT/(name+'.log');log.write_text('',encoding='utf-8');serial=OUT/(name+'.serial')
    env=os.environ.copy();env['PYTHONPATH']=str(BUILD/'klipper-python')
    process=subprocess.run(['python3',str(ROOT/'vendor/klipper/klippy/klippy.py'),str(cfg),'-i',str(commands),
                    '-o',str(serial),'-d',str(BUILD/'klipper-mcu/klipper.dict'),'-l',str(log)],env=env,check=False,timeout=120)
    messages=log.read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'.*(?:Internal error|Error loading|Traceback|Unknown command|Move exceeds|move exceeds|Extrude below|Malformed|Must home|not ready).*',messages)
    if expect_error:
        if expect_error.lower() not in messages.lower():raise RuntimeError(name+': expected rejection not observed')
        return {'case':name,'expectedRejection':expect_error,'passed':True}
    if process.returncode:raise RuntimeError(name+': native host exited '+str(process.returncode))
    if errors:raise RuntimeError(name+': '+str(errors[:3]))
    dump=subprocess.check_output(['python3',str(ROOT/'vendor/klipper/klippy/parsedump.py'),str(BUILD/'klipper-mcu/klipper.dict'),str(serial)],text=True)
    (OUT/(name+'.txt')).write_text(dump,encoding='utf-8')
    counts={}
    for oid,count in re.findall(r'queue_step oid=(\d+) interval=\d+ count=(\d+)',dump):counts[oid]=counts.get(oid,0)+int(count)
    if not counts:raise RuntimeError(name+': no motion commands generated')
    if require_rotary:
        steppers=dict(re.findall(r'config_stepper oid=(\d+) step_pin=([^\s]+)',dump))
        for pin in ['gpiochip0/gpio12','gpiochip0/gpio14','gpiochip0/gpio9']:
            ids=[oid for oid,p in steppers.items() if p==pin]
            if not ids or not counts.get(ids[0]):raise RuntimeError(name+': no queued steps for '+pin)
    return {'case':name,'passed':True,'queuedStepsByObject':counts,'physicalPrintingValidated':False}

def main():
    shutil.copy2(ROOT/'controllers/klipper/fiveaxis_flow.py',ROOT/'vendor/klipper/klippy/extras/fiveaxis_flow.py')
    config=(ROOT/'tests/klipper-batch.cfg').read_text();gcode=(ROOT/'tests/klipper-batch.gcode').read_text()
    results=[run('rotary-extension',config,gcode,require_rotary=True),
             run('stock-native-axes',config.replace('[fiveaxis_flow]',''),re.sub(r'^FIVEAXIS_ENABLE.*\n','',gcode,flags=re.M),require_rotary=True),
             run('extension-cross-section-rejection',config,gcode.replace('E0.05','E10'),expect_error='maximum deposition cross-section')]
    for filename,name in [('reference-klipper.gcode','generated-bed-job'),('head-sync.gcode','generated-head-job')]:
        generated=ROOT/'examples'/filename
        if not generated.exists():continue
        code=generated.read_text();meta=re.search(r'; FIVEAXIS_METADATA ([^\s]+)',code)
        metadata=json.loads(base64.b64decode(meta[1]));initial=metadata.get('initialCarriage',metadata['initial']['joint']['xyz'])
        prefix='SET_KINEMATIC_POSITION '+' '.join(k+'='+str(v) for k,v in zip('XYZ',initial))+'\n'
        results.append(run(name,config,prefix+code))
    (OUT/'summary.json').write_text(json.dumps({'klipperRevision':'e2b99a9159f992169c1ba53029dec020e850cc99','nativeHostBatch':results,
                'physicalPrintingValidated':False,'heaterQualification':'Klipper debug-file mode bypasses heater readiness; cold guard is covered by unit tests only.'},indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
