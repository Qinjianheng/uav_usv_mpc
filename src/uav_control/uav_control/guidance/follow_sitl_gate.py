"""Fail closed unless the task-owned local SITL harness is alive and matches this graph."""
import json
import os
from pathlib import Path
import time


def require_isolated_sitl(environ=None):
    """Validate a local harness grant, never a hardware qualification or remote attestation."""
    env = os.environ if environ is None else environ
    try:
        path = Path(env['UAV_USV_SITL_GRANT']).resolve()
        root = Path(env['UAV_USV_RESEARCH_DIRECTORY']).resolve()
        if path.parent != root or not root.is_relative_to(
                Path('/home/qin/data/uav_usv_mpc/data/experiments')):
            raise ValueError('DIRECTORY_MISMATCH')
        grant = json.loads(path.read_text())
        if (str(grant['domain']) != env['ROS_DOMAIN_ID']
                or grant['partition'] != env['GZ_PARTITION']
                or not 0 <= time.time()-grant['issued'] < 300
                or not grant['preflight_clear']):
            raise ValueError('SESSION_MISMATCH')
        stat = Path(f"/proc/{grant['pid']}/stat").read_text()
        if stat[stat.rfind(')')+2:].split()[19] != grant['process_start']:
            raise ValueError('HARNESS_NOT_ALIVE')
        if any(Path('/dev').glob('ttyACM*')) or any(Path('/dev').glob('ttyUSB*')):
            raise ValueError('SERIAL_HARDWARE_PRESENT')
        found = False
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():
                continue
            try:
                exe = os.readlink(p/'exe')
                values = (p/'environ').read_bytes().split(b'\0')
                if ('px4_sitl_default' in exe and Path(exe).name == 'px4'
                        and ('GZ_PARTITION='+grant['partition']).encode() in values):
                    found = True
            except OSError:
                pass
        if not found:
            raise ValueError('MATCHING_PX4_SITL_NOT_FOUND')
        return grant
    except (KeyError, OSError, ValueError, TypeError) as error:
        raise RuntimeError('SITL_EXECUTION_NOT_GRANTED: '+str(error)) from error
