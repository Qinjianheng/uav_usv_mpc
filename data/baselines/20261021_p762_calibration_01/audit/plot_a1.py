"""Render actual logged A1 signals; never synthesize an A2 curve."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path('/home/qin/data/uav_usv_mpc/data/experiments')
run=root/'20261021_p761_01/session_A1_03';out=root/'20261021_p762_01'
rows=[json.loads(l) for l in (run/'calibration.jsonl').read_text().splitlines()]
metrics=json.loads((out/'first_failure_analysis.json').read_text())
analysis=json.loads((run/'analysis.json').read_text());identity=json.loads((run/'identity.json').read_text())
start=next(r['stamp'] for r in rows if r['kind']=='phase');failure=metrics['failure_epoch']-start
states=[r for r in rows if r['kind']=='state'];thrust=[r for r in rows if r['kind']=='executed_collective']
t=np.array([r['stamp']-start for r in states]);z=-np.array([r['p'][2] for r in states]);vz=np.array([r['v'][2] for r in states])
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
axes[0].plot(t,z,label='Native PX4 -z (local datum)');axes[0].set_ylabel('Local height (m)')
axes[1].plot(t,vz,label='Native down velocity');axes[1].axhline(-7,color='r',ls=':',label='7 m/s speed bound');axes[1].set_ylabel('Down velocity (m/s)')
axes[2].plot([r['stamp']-start for r in thrust],[-r['xyz'][2] for r in thrust],label='PX4 executed collective');axes[2].set_ylabel('Normalized collective');axes[2].set_xlabel('Time from controller start (s)')
for ax in axes:ax.axvline(failure,color='r',ls='--',label='First safety failure');ax.grid(alpha=.25);ax.legend(loc='best',fontsize=8)
fig.suptitle('A1_03 actual dynamics: aborted, landing/disarm not achieved')
fig.savefig(out/'A1_03_dynamics.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
relative=np.array([r['stamp'] for r in thrust])-metrics['failure_epoch']
mask=(relative>=-2)&(relative<=.4)
axes[0].plot(relative[mask],np.array([-r['xyz'][2] for r in thrust])[mask]);axes[0].axvline(-.218878508,color='k',ls='--',label='RESET requested');axes[0].axvline(0,color='r',ls='--',label='Speed bound violated');axes[0].set(xlabel='Time from first failure (s)',ylabel='Executed normalized collective',title='Full collective persists after RESET');axes[0].legend(fontsize=8)
curve=analysis['measured_curve'];axes[1].plot([p['normalized'] for p in curve],[p['force'] for p in curve],'o-',label='Fixed-window measured median');axes[1].axhline(identity['model']['total_mass']*identity['model']['gravity']*2.85,color='r',ls=':',label='TWR 2.85 threshold');axes[1].set(xlabel='Body-rate collective command',ylabel='Effective lift (N)',title='A1 thrust curve does not qualify');axes[1].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(out/'A1_03_transition_and_force.png',dpi=150);plt.close(fig)
