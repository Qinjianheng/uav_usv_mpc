import json, os, signal, subprocess, sys, time
from pathlib import Path
session=Path(sys.argv[1]);output=Path(sys.argv[2]);output.mkdir(parents=True,exist_ok=True)
rows=subprocess.check_output(['ps','-eo','pid=,ppid=,pgid=,args='],text=True).splitlines()
records={}
for r in rows:
 parts=r.strip().split(None,3)
 if len(parts)==4:records[int(parts[0])]={'ppid':int(parts[1]),'pgid':int(parts[2]),'args':parts[3]}
roots=[int(p.read_text()) for p in session.glob('*.pid')]
owned=set(roots)
for _ in range(12):
 owned.update(p for p,r in records.items() if r['ppid'] in owned)
owned.discard(os.getpid());session.joinpath('restarting').touch()
with (output/'owned_processes_before_cleanup.json').open('w') as f:json.dump({p:records.get(p) for p in sorted(owned)},f,indent=2)
for p in sorted(owned,reverse=True):
 try:os.kill(p,signal.SIGINT)
 except ProcessLookupError:pass
for _ in range(40):
 alive=[p for p in owned if Path(f'/proc/{p}').exists()]
 if not alive:break
 time.sleep(.1)
for p in alive:
 try:os.kill(p,signal.SIGTERM)
 except ProcessLookupError:pass
print('Stopped owned roots',roots,'remaining',alive)
