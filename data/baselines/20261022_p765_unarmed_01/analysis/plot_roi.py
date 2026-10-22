"""Display the measured ROI and five far-background depth pixels; no input edits."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).parent
ROI=HERE.parent/'nonflight_01/roi'
meta=json.loads(sorted(ROI.glob('roi_*.json'))[0].read_text())
with np.load(ROI/meta['data_file'],allow_pickle=False) as packet:
    rgb,depth,mask=packet['rgb'],packet['depth'],packet['mask']
valid=mask & np.isfinite(depth)&(depth>=.05)&(depth<=25)
values=np.sort(depth[valid]);groups=np.split(values,np.flatnonzero(np.diff(values)>.5)+1)
fg=max(groups,key=len);background=valid&((depth<fg[0])|(depth>fg[-1]))
y,x=np.where(background)
fig,axes=plt.subplots(1,2,figsize=(8,4),layout='constrained')
axes[0].imshow(rgb,interpolation='nearest');axes[0].contour(mask,levels=[.5],colors='white',linewidths=.8)
axes[0].scatter(x,y,s=75,facecolors='none',edgecolors='#ffbf00',linewidths=1.7)
axes[0].set_title('RGB + YOLO mask\nYellow rings: 5 blue background pixels')
show=np.where(mask,depth,np.nan)
im=axes[1].imshow(show,interpolation='nearest',vmin=6.3,vmax=25.,cmap='viridis')
axes[1].scatter(x,y,s=75,facecolors='none',edgecolors='#ffbf00',linewidths=1.7)
axes[1].set_title('Original forward-X depth (m)\n263 sphere points + 5 far points')
fig.colorbar(im,ax=axes[1],shrink=.75,label='metres')
for ax in axes:
    ax.set_xlabel('ROI column');ax.set_ylabel('ROI row')
fig.savefig(HERE/'roi_background_evidence.png',dpi=170)
