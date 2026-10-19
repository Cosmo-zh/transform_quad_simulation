from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

repo = Path(__file__).resolve().parents[2]
log = repo / 'build/px4_sitl_default/rootfs/log/2026-10-12/05_55_42.ulg'
u = ULog(str(log))
out = Path(__file__).resolve().parent
def data(name):
    d = u.get_dataset(name).data
    return (d['timestamp'].astype(float)-u.start_timestamp)/1e6, d
tp,p = data('vehicle_local_position')
ts,s = data('trajectory_setpoint')
tg,g = data('vehicle_local_position_groundtruth')
tm,m = data('manual_control_setpoint')
ta,a = data('vehicle_attitude')
tv,v = data('vehicle_status')
neutral = np.maximum.reduce([np.abs(m[k]) for k in ['roll','pitch','yaw','throttle']]) < .02
edges = np.diff(np.r_[False,neutral,False].astype(int))
intervals = []
for i,j in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
    start,end=tm[i],tm[min(j,len(tm)-1)]
    if start<49 or end-start<5:
        continue
    left=start+3
    mask=(tp>=left)&(tp<=end)
    if not np.any(mask): continue
    xsp=np.interp(tp[mask],ts,s['position[0]']); ysp=np.interp(tp[mask],ts,s['position[1]'])
    if not np.all(np.isfinite(xsp+ysp)): continue
    error=np.hypot(p['x'][mask]-xsp,p['y'][mask]-ysp)
    gm=(tg>=left)&(tg<=end)
    am=(ta>=left)&(ta<=end)
    w,x,y,z=[a[f'q[{k}]'][am] for k in range(4)]
    roll=np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    pitch=np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
    intervals.append(dict(neutral_start=float(start),neutral_end=float(end),analysis_start=float(left),
        samples=int(mask.sum()),horizontal_error_max_m=float(error.max()),horizontal_error_rms_m=float(np.sqrt(np.mean(error**2))),
        estimated_cross_track_span_m=float(np.ptp(p['x'][mask])),truth_cross_track_span_m=float(np.ptp(g['x'][gm])),
        truth_along_track_span_m=float(np.ptp(g['y'][gm])),speed_max_m_s=float(np.max(np.hypot(p['vx'][mask],p['vy'][mask]))),
        roll_abs_max_deg=float(np.max(np.abs(roll))),pitch_abs_max_deg=float(np.max(np.abs(pitch))),
        setpoint_span_m=[float(np.ptp(xsp)),float(np.ptp(ysp))]))
summary={'log':str(log.relative_to(repo)),'duration_s':float((u.last_timestamp-u.start_timestamp)/1e6),
         'dropouts':len(u.dropouts),'neutral_threshold':.02,'excluded_braking_s':3,'hover_intervals':intervals,
         'position_reset_counters':{k:np.unique(p[k]).tolist() for k in ['xy_reset_counter','z_reset_counter','heading_reset_counter']},
         'failsafe':bool(np.any(v['failsafe']))}
(out/'metrics.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
fig,axs=plt.subplots(5,1,figsize=(13,13),sharex=True)
for ax,key,idx,title in [(axs[0],'x',0,'Cross-track / NED X (m)'),(axs[1],'y',1,'Along-track / NED Y (m)')]:
    ax.plot(tp,p[key],label='EKF estimate');ax.plot(ts,s[f'position[{idx}]'],label='Position target',ls='--')
    ax.plot(tg,g[key],label='Gazebo truth (own origin)',alpha=.7);ax.set_ylabel(title);ax.legend(loc='upper left')
axs[2].plot(tp,np.hypot(p['vx'],p['vy']),label='Estimated horizontal speed')
axs[2].plot(tg,np.hypot(g['vx'],g['vy']),label='Truth horizontal speed',alpha=.7)
axs[2].set_ylabel('Speed (m/s)');axs[2].legend()
for key in ['roll','pitch','yaw','throttle']: axs[3].plot(tm,m[key],label=key)
axs[3].set_ylabel('Manual input');axs[3].legend(ncol=4)
axs[4].plot(tp,-p['z'],label='Estimated altitude');axs[4].plot(ts,-s['position[2]'],label='Altitude target',ls='--')
axs[4].set_ylabel('Altitude (m)');axs[4].set_xlabel('Seconds from log start');axs[4].legend()
for ax in axs:
    ax.grid(alpha=.25);ax.set_xlim(30,105)
    for segment in intervals:ax.axvspan(segment['analysis_start'],segment['neutral_end'],color='green',alpha=.12)
fig.suptitle('5102 Position flight: neutral-stick windows after 3 s braking exclusion')
fig.tight_layout();fig.savefig(out/'position_drift.png',dpi=160)
