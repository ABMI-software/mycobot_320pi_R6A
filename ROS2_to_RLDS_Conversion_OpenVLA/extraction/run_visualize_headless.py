"""Runs the repo's own, unmodified visualize_dataset.py in a headless
container: no display is available for plt.show(), so this monkeypatches
plt.show() to save every open figure to disk instead, then executes the
real script via runpy. The validation logic itself is untouched -- this
only swaps "open a window" for "save a PNG" so the results can be looked
at afterward instead of live."""
import os
import sys

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.makedirs('/workspace/rlds_extraction_out/vis', exist_ok=True)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def _save_all(*_args, **_kwargs):
    for num in plt.get_fignums():
        fig = plt.figure(num)
        title = fig.axes[0].get_title() if fig.axes else str(num)
        safe = "".join(c if c.isalnum() else "_" for c in str(title))[:60]
        out = f"/workspace/rlds_extraction_out/vis/fig_{num}_{safe}.png"
        fig.savefig(out, dpi=110, bbox_inches='tight')
        print("saved", out)

plt.show = _save_all

sys.argv = ['visualize_dataset.py', 'gazebo_to_lerobot_mycobot']
sys.path.insert(0, '/workspace/rlds_dataset_builder')
import runpy
runpy.run_path('/workspace/rlds_dataset_builder/visualize_dataset.py', run_name='__main__')
