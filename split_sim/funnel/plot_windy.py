"""Plot independently measured paths and errors for a recorded batch."""
import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from summarize_windy import summary


def plot(batch):
    records=summary(batch)['runs']
    paths=batch.parents[1]
    if not records:
        return
    fig,(ax,err)=plt.subplots(1,2,figsize=(9.5,3.7),layout='constrained',
                               gridspec_kw={'width_ratios':[1.12,1]})
    models=ET.parse(paths/records[0]['run_id']/'funnel.sdf')
    for name in ('left','right'):
        wall=models.find(f"world/model[@name='{name}']")
        x,y,_,_,_,angle=map(float,wall.findtext('pose').split())
        length,width,_=map(float,wall.findtext('link/collision/geometry/box/size').split())
        points=[(x+i*length*math.cos(angle)-j*width*math.sin(angle),
                 y+i*length*math.sin(angle)+j*width*math.cos(angle))
                for i,j in ((-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5))]
        ax.add_patch(Polygon(points,color='#3f4750'))
    for run in records:
        samples=json.loads((paths/run['run_id']/'flight/track.json').read_text())['samples']
        airborne=[s for s in samples if s['armed'] and not s['landed'] and s['status_age']<=1.5]
        color='#126e8b' if run['wind_mean_y_m_s']>0 else '#b75128'
        ax.plot([s['x'] for s in airborne],[s['y'] for s in airborne],lw=1,alpha=.72,color=color)
        if run['lateral_error_m'] is not None:
            err.scatter(run['index'],100*run['lateral_error_m'],s=28,color=color)
        else:
            err.plot(run['index'],0,'x',color='#353535')
    ax.set(xlim=(-.35,5.45),ylim=(-2.65,2.65),aspect='equal',xlabel='East (m)',ylabel='North (m)',title='Recorded paths through the funnel')
    err.axhline(0,color='.55',lw=.8)
    for color,label in [('#126e8b','Wind north'),('#b75128','Wind south')]:
        err.scatter([],[],s=24,color=color,label=label)
    err.legend(frameon=False,loc='best',fontsize=8)
    err.set(xlim=(.3,20.7),xticks=[1,5,10,15,20],xlabel='Run',ylabel='North of opening midpoint (cm)',title='Lateral offset at crossing')
    for axis in (ax,err):
        axis.spines[['top','right']].set_visible(False)
    fig.savefig(batch/'crossings.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    plot(Path(sys.argv[1]))
