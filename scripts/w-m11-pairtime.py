# yack: interleaved timing of two yack-step measure binaries over the ladder33 STEP files.
# usage: python3 scripts/w-m11-pairtime.py A B
# Interleaved timing: each file measured by A then B (order alternating), summed, so load drift cancels.
import json,os,subprocess,sys,time
R='/home/brenn/github/yack-ref'; M=R+'/brep-bench/results/meshes/yack'
d=json.load(open(R+'/runs/brep-bench/yack-ladder33.json'))
files=[f"{M}/{c['case']}.step" for c in d['cases'] if c.get('measured')=='step' and os.path.exists(f"{M}/{c['case']}.step")]
A,B=sys.argv[1],sys.argv[2]; ta=tb=0.0
for i,f in enumerate(files):
    for b in ((A,B) if i%2==0 else (B,A)):
        t0=time.time(); subprocess.run([b,f],capture_output=True); dt=time.time()-t0
        if b==A: ta+=dt
        else: tb+=dt
print(f"files {len(files)} interleaved: {os.path.basename(A)} {ta:.1f} s, {os.path.basename(B)} {tb:.1f} s, ratio {tb/ta:.3f}")
