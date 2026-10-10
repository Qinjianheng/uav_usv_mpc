"""Bound one authorized launcher session, preserving exact component PID identities."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

root = Path(sys.argv[1]).resolve()
lightweight = sys.argv[2]
started = time.time()
env = dict(os.environ, UAV_USV_WS=str(root.parents[3]),
           UAV_USV_RESEARCH_DIRECTORY=str(root), UAV_USV_RESEARCH_LIGHTWEIGHT=lightweight,
           UAV_USV_EXPERIMENT_LAUNCH='follow_research.launch.py',
           UAV_USV_EXPERIMENT_CONFIG_FILE=str(root/'config/flight.yaml'),
           ROS_LOG_DIR=str(root/'ros_logs'), CAMERA_STARTUP_TIMEOUT='90',
           FLIGHT_READY_TIMEOUT='60', UAV_USV_RESEARCH_CONFIG_FILE=str(root/'config/research.yaml'))
workspace=Path('/home/qin/data/uav_usv_mpc')
env['UAV_USV_WS']=str(workspace)
console=(root/'launcher.txt').open('x')
launcher=subprocess.Popen([str(workspace/'scripts/uav_lab.sh'), '--no-build'], cwd=workspace,
                          env=env, stdin=subprocess.PIPE, stdout=console, stderr=console,
                          start_new_session=True)
session=None
identities={}
command_sent=False
try:
    while time.time()-started < (int(sys.argv[3]) if len(sys.argv)>3 else 140) and not (root/'STOP').exists():
        if session is None:
            candidates=[p for p in Path('/tmp').glob('uav_usv_lab.*') if p.stat().st_mtime>=started]
            if len(candidates)==1:
                session=candidates[0]
        if session:
            for p in session.glob('*.pid'):
                try:
                    pid=int(p.read_text())
                    stat=Path(f'/proc/{pid}/stat').read_text().split()
                    identities.setdefault(p.stem,dict(pid=pid,pgid=os.getpgid(pid),start=stat[21]))
                except (ValueError,FileNotFoundError,ProcessLookupError):
                    pass
            (root/'owned_session.json').write_text(json.dumps(dict(started=started,
                launcher_pid=launcher.pid, session=str(session), components=identities),indent=2))
        if not command_sent and (root.name == 'f2' or root.name.startswith('f3')) and 'Two-stage control is ready' in (root/'launcher.txt').read_text():
            command_sent=True
            with (root/'X_command.txt').open('x') as log:
                subprocess.run(['bash','-c', 'source /opt/ros/humble/setup.bash; source /home/qin/data/uav_usv_mpc/install/setup.bash; timeout 8s ros2 topic pub --times 3 --rate 10 --print 3 --wait-matching-subscriptions 2 --keep-alive 0.5 /simulation/impact/command std_msgs/msg/String \"{data: X}\"'], env=env, stdout=log, stderr=log, timeout=12)
        time.sleep(.5)
finally:
    for sig, delay in ((signal.SIGINT,6),(signal.SIGTERM,3),(signal.SIGHUP,1)):
        for entry in identities.values():
            try:
                stat=Path(f"/proc/{entry['pid']}/stat").read_text().split()
                if stat[21]==entry['start'] and os.getpgid(entry['pid'])==entry['pgid']:
                    os.killpg(entry['pgid'],sig)
            except (FileNotFoundError,ProcessLookupError):
                pass
        time.sleep(delay)
    launcher.terminate()
    try:
        launcher.wait(3)
    except subprocess.TimeoutExpired:
        launcher.kill()
    console.close()
    (root/'watchdog_end.json').write_text(json.dumps(dict(started=started,ended=time.time(),
        components=identities,reason='STOP' if (root/'STOP').exists() else 'wall bound'),indent=2))
    print('owned session stopped',root,flush=True)
