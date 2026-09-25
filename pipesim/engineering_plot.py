"""Standalone scientific stress/deflection figure for the build book."""
def engineering_plot(assembly,result,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    import numpy as np
    from .math3d import point
    fig=plt.figure(figsize=(10,7),layout='constrained'); ax=fig.add_subplot(111,projection='3d')
    values={m['part']:m for m in result['members']}
    vmax=max((m['max_von_mises_mpa'] for m in values.values()),default=1)
    norm=Normalize(0,max(vmax,1e-6)); cmap=plt.get_cmap('viridis')
    max_displacement=max((m['max_displacement_mm'] for m in values.values()),default=0)
    scale=min(1000,50/max_displacement) if max_displacement>1e-9 else 1
    nodes=np.array([n['position_mm'] for n in result['nodes']]); shifts=np.array([n['translation_mm'] for n in result['nodes']])
    for pid,m in values.items():
        p=assembly.parts[pid]
        for element in m['elements']:
            ends=np.array([point(p.matrix,[0,0,s-p.length/2]) for s in (element['from_mm'],element['to_mm'])])
            ids=[np.linalg.norm(nodes-e,axis=1).argmin() for e in ends]
            deformed=ends+scale*shifts[ids]
            ax.plot(*ends.T,color=cmap(norm(element['von_mises_mpa'])),linewidth=3)
            ax.plot(*deformed.T,color='#d86e49',linestyle='--',linewidth=1.5)
        center=p.matrix[:3,3]; ax.text(*center,pid,fontsize=7)
    ax.set(xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)',title=f'Linear frame response · displacement ×{scale:.1f}')
    ax.view_init(25,-55)
    extent=np.ptp(nodes,axis=0); ax.set_box_aspect(np.maximum(extent,max(extent.max(),1)*.2))
    fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=cmap),ax=ax,shrink=.65,label='Equivalent stress (MPa)')
    fig.savefig(path,dpi=150,facecolor='white'); plt.close(fig)
    return str(path)

def layout_plot(assembly,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from .geometry import bounds
    fig,axes=plt.subplots(3,1,figsize=(8,10),layout='constrained')
    for ax,(i,j),title in zip(axes,[(0,1),(0,2),(1,2)],['Top · X / Y','Front · X / Z','Side · Y / Z']):
        for pid,part in assembly.parts.items():
            box=bounds(part)
            ax.add_patch(Rectangle((box[0,i],box[0,j]),box[1,i]-box[0,i],box[1,j]-box[0,j],edgecolor='#a3b3bd',facecolor='none',linewidth=.7))
        for anchor in assembly.anchors:
            p=assembly.parts[anchor['part']].matrix[:3,3]
            ax.plot(p[i],p[j],'o',color='#b1623f',markersize=4)
            ax.annotate(anchor['part'],(p[i],p[j]),xytext=(5,5),textcoords='offset points',fontsize=7)
        ax.autoscale_view(); ax.margins(.15); ax.set_aspect('equal',adjustable='datalim')
        ax.set(title=title,xlabel='XYZ'[i]+' (mm)',ylabel='XYZ'[j]+' (mm)'); ax.grid(alpha=.2)
        ax.axhline(0,color='#627985',linewidth=.5); ax.axvline(0,color='#627985',linewidth=.5)
    fig.savefig(path,dpi=150,facecolor='white'); plt.close(fig)
