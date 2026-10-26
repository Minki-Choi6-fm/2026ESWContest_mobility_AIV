#!/usr/bin/env python3
"""Regression: full pipeline ESP-NOW path using only fake STM32 and bridge ptys."""
import csv
from pathlib import Path
import subprocess
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parent))
from espnow_bridge_sim_test import Esp32Model, FRONT_DIR
HERE=Path(__file__).resolve().parent
OUT=HERE.parents[1]/'logs/codex1'/('pipeline_espnow_'+time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir(parents=True)
A,B=Esp32Model('rear'),Esp32Model('front');A.peer=B;B.peer=A
procs=[];lines=[]
try:
    front=subprocess.Popen([sys.executable,'-u','front_node.py','--no-stm32','--transport','esp-now','--serial-port',B.path],cwd=FRONT_DIR,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    procs.append(front)
    threading.Thread(target=lambda:lines.extend(front.stdout),daemon=True).start()
    link=OUT/'stm32.pty'
    sim=subprocess.Popen([sys.executable,'-I','sim_scenario.py','--link',str(link),'--duration','10'],cwd=HERE,stdout=subprocess.DEVNULL)
    procs.append(sim)
    for _ in range(50):
        if link.exists():break
        time.sleep(.1)
    r=subprocess.run([sys.executable,'-I','pipeline_main.py','--uart',str(link),'--transport','esp-now','--serial-port',A.path,'--out',str(OUT),'--duration','6','--buzzer','mock','--sinks','csv,big'],cwd=HERE,capture_output=True,text=True,timeout=20)
    (OUT/'pipeline.log').write_text(r.stdout+r.stderr)
    rows=list(csv.DictReader(x for x in (OUT/'drive_log.csv').open() if not x.startswith('#')))
    assert r.returncode==0,r.stderr
    assert len(rows)>80,len(rows)
    assert A.tx_ok>30 and A.rx_ok>0,(A.tx_ok,A.rx_ok)
    assert any('후방차=LIVE' in x for x in lines),lines[-5:]
    assert 'pin OFF' in r.stdout,r.stdout[-1000:]
    print(f'PASS full pipeline ESP-NOW pty: {len(rows)} evaluations, rear TX={A.tx_ok}, peer RX={A.rx_ok}, front LIVE, mock OFF')
    print('Logs:',OUT)
finally:
    for p in procs:p.terminate()
    for p in procs:
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    A.stop=B.stop=True
