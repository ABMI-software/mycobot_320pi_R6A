"""A trained SmolVLA checkpoint as one call: three RGB images, the state, the task -> 50 actions.

Shared by open_loop_check.py (recorded frames) and eval_smolvla_tri_sort.py (live Gazebo), so
both feed the model exactly the same way. The saved preprocessor does the rest as in training:
camera renaming top/right/left -> camera1/2/3, tokenisation, MEAN_STD state normalisation; the
postprocessor turns the actions back into radians (6 joints) and the gripper command.

Runs in /workspace/venv_lerobot (lerobot 0.6.1, the training versions, CPU torch).
"""
from pathlib import Path

import numpy as np
import torch
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

CAMERAS = ('top', 'right', 'left')


class SmolVLA:
    def __init__(self, checkpoint):
        self.checkpoint = Path(checkpoint)
        self.policy = SmolVLAPolicy.from_pretrained(str(self.checkpoint))
        self.policy.eval()
        # The checkpoints were saved on a Kaggle GPU: their device step says cuda.
        self.pre, self.post = make_pre_post_processors(
            self.policy.config, str(self.checkpoint),
            preprocessor_overrides={'device_processor': {'device': 'cpu'}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}})

    @torch.no_grad()
    def predict(self, images, state, task, seed=0):
        """images: {camera: HxWx3 uint8 RGB}; state: 7 floats (6 joints rad + gripper).

        Returns the (chunk_size, 7) action chunk. Flow matching starts from noise: the seed
        makes a chunk reproducible.
        """
        obs = {f'observation.images.{c}': torch.from_numpy(np.ascontiguousarray(images[c]))
               .permute(2, 0, 1).float() / 255.0 for c in CAMERAS}
        obs['observation.state'] = torch.as_tensor(np.asarray(state, dtype=np.float32))
        obs['task'] = task
        self.policy.reset()
        torch.manual_seed(seed)
        actions = self.policy.predict_action_chunk(self.pre(obs))
        return self.post(actions)[0].numpy()

    @torch.no_grad()
    def predict_frame(self, frame, seed=0):
        """Same, from a LeRobotDataset item (CHW float images in [0, 1], as in training)."""
        images = {c: (frame[f'observation.images.{c}'].permute(1, 2, 0).numpy() * 255.0)
                  .round().astype(np.uint8) for c in CAMERAS}
        return self.predict(images, frame['observation.state'].numpy(), frame['task'], seed)
