#!/usr/bin/env bash
# Produces the numbers in MEASUREMENTS.md §1-2: real-time factor and per-core
# CPU, server only (headless, no GUI), episode-1 scene spawned, arm idle, for
# four camera configurations. Run inside the container from /workspace/htgspp:
#   bash scripts/measure_perf.sh [seconds_per_variant]
# Variants are temporary copies in the INSTALLED mycobot_description share;
# the installed URDF is backed up and restored on exit, whatever happens.
set -u
DUR=${1:-30}
SHARE=/workspace/install/mycobot_description/share/mycobot_description
URDF=$SHARE/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf
cp "$URDF" /tmp/urdf.orig
cleanup() {
  cp /tmp/urdf.orig "$URDF"
  rm -f "$SHARE"/worlds/perf_*.sdf
  pkill -INT -f "[s]im_grasp.launch.py"; sleep 4
  for p in "[g]z sim" "[r]os2 launch" "[p]arameter_bridge" "[r]obot_state_publisher"; do pkill -9 -f "$p"; done
}
trap cleanup EXIT
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; set -u

W=$SHARE/worlds/sorting_table.sdf
python3 - "$W" "$SHARE/worlds" <<'EOF'
import re, sys
w, out = open(sys.argv[1]).read(), sys.argv[2]
def save(name, text):
    open(f"{out}/{name}.sdf", "w").write(text.replace('<world name="sorting_table">', f'<world name="{name}">'))
save("perf_base", w)
save("perf_nocam", re.sub(r"<sensor name=\"table_camera\".*?</sensor>", "", w, flags=re.S))
save("perf_lowres", w.replace("<width>1280</width>", "<width>320</width>").replace("<height>960</height>", "<height>240</height>"))
EOF

run_variant() {   # $1 label  $2 world  $3 urdf-mode (keep|nosensors)
  if [ "$3" = nosensors ]; then
    python3 -c "
import re; s=open('/tmp/urdf.orig').read()
open('$URDF','w').write(re.sub(r'<plugin filename=\"gz-sim-sensors-system\".*?</plugin>', '', s, flags=re.S))"
  else cp /tmp/urdf.orig "$URDF"; fi
  DISPLAY=:0 nohup ros2 launch mycobot_gateway sim_grasp.launch.py world_name:="$2" headless:=true \
     bridge_camera:=false > "/tmp/perf_$1.log" 2>&1 &
  for i in $(seq 1 40); do
    [ "$(timeout 15 ros2 control list_controllers 2>/dev/null | grep -cw active)" -ge 3 ] && break; sleep 3
  done
  python3 - "$2" <<'EOF' >/dev/null 2>&1
import subprocess, sys, csv, yaml
world = sys.argv[1]
cfg = yaml.safe_load(open("config/objects.yaml"))
row = next(r for r in csv.DictReader(open("config/episode_matrix.csv")) if r["episode"] == "1")
for bn, b in cfg["bins"].items():
    subprocess.run(["ros2","run","ros_gz_sim","create","-world",world,"-file",f"/workspace/htgspp/models/{bn}.sdf","-name",bn,"-x",str(b["pose"][0]),"-y",str(b["pose"][1]),"-z","0"],timeout=90)
for n in cfg["objects"]:
    subprocess.run(["ros2","run","ros_gz_sim","create","-world",world,"-file",f"/workspace/htgspp/models/{n}.sdf","-name",n,"-x",row[f"{n}_x"],"-y",row[f"{n}_y"],"-z","0.03"],timeout=90)
    subprocess.run(["gz","topic","-t",f"/htgspp/{n}/detach","-m","gz.msgs.Empty","-p","unused: false"],timeout=10)
EOF
  sleep 10
  python3 - "$1" "$2" "$DUR" <<'EOF'
import re, subprocess, sys, time, statistics
label, world, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
def cpu():
    rows = [l.split() for l in open("/proc/stat") if re.match(r"cpu\d", l)]
    return [(sum(map(int, r[1:])), int(r[4]) + int(r[5])) for r in rows]
def gzcpu():
    out = subprocess.run(["ps", "-eo", "pcpu,args"], capture_output=True, text=True).stdout
    return sum(float(l.split()[0]) for l in out.splitlines()[1:] if "gz sim" in l)
c0, rtfs, t0 = cpu(), [], time.time()
while time.time() - t0 < dur:
    out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/stats"],
                         capture_output=True, text=True, timeout=10).stdout
    m = re.search(r"real_time_factor:\s*([\d.]+)", out)
    if m: rtfs.append(float(m.group(1)))
    time.sleep(1)
c1 = cpu()
busy = [100 * (1 - (i1 - i0) / max(t1 - t0, 1)) for (t0, i0), (t1, i1) in zip(c0, c1)]
print(f"{label:<28} rtf mean {statistics.mean(rtfs):.3f} (min {min(rtfs):.3f}, max {max(rtfs):.3f}, n={len(rtfs)})"
      f" | per-core busy % {[round(b) for b in busy]} | total {statistics.mean(busy):.0f}%"
      f" | gz sim ps %CPU {gzcpu():.0f}", flush=True)
EOF
  pkill -INT -f "[s]im_grasp.launch.py"; sleep 5
  for p in "[g]z sim" "[r]os2 launch" "[p]arameter_bridge" "[r]obot_state_publisher"; do pkill -9 -f "$p"; done
  sleep 3
}

run_variant "A all cameras on (as-is)"   perf_base   keep
run_variant "B table camera removed"     perf_nocam  keep
run_variant "C table cam 320x240"        perf_lowres keep
run_variant "D no camera rendering"      perf_nocam  nosensors
