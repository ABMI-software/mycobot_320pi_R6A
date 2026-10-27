"""Share of the 120 chunks whose predicted close is within 20 mm of the recorded close (grasp_point_mm),
other minus reference, 95 % interval from 2000 resamples of whole episodes (open_loop_compare's scheme)."""
import json, sys
import numpy as np

LIMIT = 20.0


def load(path):
    rows = json.load(open(path))['chunks']
    return {(r['episode'], r['start']): r.get('grasp_point_mm', np.inf) <= LIMIT for r in rows}


ref_path = sys.argv[1]
ref = load(ref_path)
episodes = sorted({k[0] for k in ref})
by_ep = {e: [k for k in ref if k[0] == e] for e in episodes}
for other_path in sys.argv[2:]:
    other = load(other_path)
    assert other.keys() == ref.keys()

    def share(d, eps):
        ks = [k for e in eps for k in by_ep[e]]
        return np.mean([d[k] for k in ks])

    rng = np.random.default_rng(0)
    draws = []
    for _ in range(2000):
        eps = rng.choice(episodes, len(episodes))
        draws.append(share(other, eps) - share(ref, eps))
    lo, hi = np.percentile(draws, [2.5, 97.5])
    print(f'{other_path.split("/")[-1]} minus {ref_path.split("/")[-1]}: within {LIMIT:.0f} mm '
          f'{sum(other.values())}/120 vs {sum(ref.values())}/120, diff {share(other, episodes) - share(ref, episodes):+.3f} '
          f'[{lo:+.3f}, {hi:+.3f}] ({len(episodes)} episodes)')
