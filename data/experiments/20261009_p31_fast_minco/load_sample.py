"""Read CPU/RSS for exact owned session groups; one core=100%, RSS sum not unique RAM."""
import json
import os
from pathlib import Path
import sys
import time

root=Path(sys.argv[1]).resolve()
label=sys.argv[2]
info=json.loads((root/'owned_session.json').read_text())
groups={v['pgid'] for v in info['components'].values()}
hz=os.sysconf('SC_CLK_TCK')

def snap():
    values={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            raw=(p/'stat').read_text()
            stat=raw[raw.rfind(')')+2:].split()
            if int(stat[2]) not in groups:
                continue
            ticks=int(stat[11])+int(stat[12])
            rss=int(stat[21])*os.sysconf('SC_PAGE_SIZE')
            command=(p/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
            values[p.name]=dict(ticks=ticks,rss=rss,command=command)
        except (OSError,ValueError):
            pass
    return values

rows=[]
previous=snap()
last=time.monotonic()
for _ in range(20):
    time.sleep(1)
    now=time.monotonic()
    current=snap()
    duration=now-last
    cpu=sum(v['ticks']-previous[k]['ticks'] for k,v in current.items() if k in previous)
    research={k:v for k,v in current.items() if '/follow_research_shadow_node ' in v['command']}
    research_cpu=sum(v['ticks']-previous[k]['ticks'] for k,v in research.items() if k in previous)
    rows.append(dict(stamp=time.time(),cpu_percent=100*cpu/hz/duration,
        rss_mib=sum(v['rss'] for v in current.values())/2**20,
        research_cpu_percent=100*research_cpu/hz/duration,
        research_rss_mib=sum(v['rss'] for v in research.values())/2**20))
    previous,last=current,now
(root/(label+'_load.json')).write_text(json.dumps(dict(groups=list(groups),samples=rows,
    processes=current),indent=2))
print(label,'CPU',sum(r['cpu_percent'] for r in rows)/len(rows),
      'RSS',sum(r['rss_mib'] for r in rows)/len(rows),flush=True)
