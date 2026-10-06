"""Exercise the real, registered gazebo_to_lerobot_mycobot_dataset_transform (and
confirm the registry entries) WITHOUT installing OpenVLA's full dependency
stack (torch/transformers/etc, multi-GB) -- that's disproportionate just to
test-import 3 config files. Instead, stub the `prismatic` package hierarchy
in sys.modules with empty placeholder modules carrying the correct
`__path__`, so Python's import machinery resolves the real oxe/*.py files
on disk without ever executing prismatic/__init__.py (which is what
actually pulls in the heavy model-loading code)."""
import sys
import types
import os

OPENVLA_ROOT = "/workspace/openvla"
sys.path.insert(0, OPENVLA_ROOT)


def _stub_package(dotted_name, real_subpath):
    mod = types.ModuleType(dotted_name)
    mod.__path__ = [os.path.join(OPENVLA_ROOT, real_subpath)]
    sys.modules[dotted_name] = mod
    return mod


_stub_package("prismatic", "prismatic")
_stub_package("prismatic.vla", "prismatic/vla")
_stub_package("prismatic.vla.datasets", "prismatic/vla/datasets")
_stub_package("prismatic.vla.datasets.rlds", "prismatic/vla/datasets/rlds")
_stub_package("prismatic.vla.datasets.rlds.oxe", "prismatic/vla/datasets/rlds/oxe")
_stub_package("prismatic.vla.datasets.rlds.oxe.utils", "prismatic/vla/datasets/rlds/oxe/utils")
_stub_package("prismatic.vla.datasets.rlds.utils", "prismatic/vla/datasets/rlds/utils")

import tensorflow as tf
import tensorflow_datasets as tfds

from prismatic.vla.datasets.rlds.oxe.transforms import (
    OXE_STANDARDIZATION_TRANSFORMS,
    gazebo_to_lerobot_mycobot_dataset_transform,
)
from prismatic.vla.datasets.rlds.oxe.configs import OXE_DATASET_CONFIGS, StateEncoding, ActionEncoding
from prismatic.vla.datasets.rlds.oxe.mixtures import OXE_NAMED_MIXTURES

assert "gazebo_to_lerobot_mycobot" in OXE_DATASET_CONFIGS
assert "gazebo_to_lerobot_mycobot" in OXE_STANDARDIZATION_TRANSFORMS
assert "gazebo_to_lerobot_mycobot" in OXE_NAMED_MIXTURES
assert OXE_STANDARDIZATION_TRANSFORMS["gazebo_to_lerobot_mycobot"] is gazebo_to_lerobot_mycobot_dataset_transform
cfg = OXE_DATASET_CONFIGS["gazebo_to_lerobot_mycobot"]
assert cfg["state_encoding"] == StateEncoding.POS_QUAT
assert cfg["action_encoding"] == ActionEncoding.EEF_POS
print("Registry entries present and consistent: configs / transforms / mixtures OK")

ds = tfds.load("gazebo_to_lerobot_mycobot", split="train")
ep = next(iter(ds))
steps = ep["steps"]

state = tf.stack([s["observation"]["state"] for s in steps])
action = tf.stack([s["action"] for s in steps])
print("input state shape/dtype:", state.shape, state.dtype)
print("input action shape/dtype:", action.shape, action.dtype)
print("input gripper (state) range:", tf.reduce_min(state[:, 7]).numpy(), tf.reduce_max(state[:, 7]).numpy())

trajectory = {"observation": {"state": state}, "action": action}
out = gazebo_to_lerobot_mycobot_dataset_transform(trajectory)

out_state = out["observation"]["state"]
out_action = out["action"]
print("\noutput state shape/dtype:", out_state.shape, out_state.dtype)
print("output action shape/dtype:", out_action.shape, out_action.dtype)
print("output gripper (state) range:", tf.reduce_min(out_state[:, 7]).numpy(), tf.reduce_max(out_state[:, 7]).numpy())
print("output gripper (action) range:", tf.reduce_min(out_action[:, 6]).numpy(), tf.reduce_max(out_action[:, 6]).numpy())

assert tf.reduce_min(out_state[:, 7]) >= 0.0 and tf.reduce_max(out_state[:, 7]) <= 1.0
assert tf.reduce_min(out_action[:, 6]) >= 0.0 and tf.reduce_max(out_action[:, 6]) <= 1.0
assert tf.reduce_max(tf.abs(out_state[:, :7] - tf.cast(state[:, :7], tf.float32))).numpy() == 0.0
assert tf.reduce_max(tf.abs(out_action[:, :6] - tf.cast(action[:, :6], tf.float32))).numpy() == 0.0
print("\nAll assertions passed: gripper correctly normalized into [0,1], other columns untouched.")
