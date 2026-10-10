from typing import Any, Iterator, Tuple

import glob
import os

import numpy as np
import tensorflow_datasets as tfds


def _image(camera, doc):
    return tfds.features.Image(shape=(224, 224, 3), dtype=np.uint8, encoding_format='jpeg',
                               doc=f'{camera} camera RGB, 640x480 resized to 224x224 without '
                                   f'crop (a centre crop cuts the board edge). {doc}')


class MycobotTriSort(tfds.core.GeneratorBasedBuilder):
    """MyCobot 320 Pi, Osama's four-object sort in Gazebo (tri_yolo scene).

    One episode = one pick-and-place: one coloured object into the bin of its
    colour, from "▶ <piece>" to its verdict in sim_sorting_grasp, recorded
    with rosetta (contract mycobot_tri_sort.yaml) and extracted by
    extraction/extract_tri_sort.py. Physical grasp (gripper contact, no weld);
    object and bin positions from Gazebo ground truth, which never appear in
    the observation. Only episodes whose verdict is OK are included.
    10 Hz, the cameras' native rate. Action = the controllers' COMMAND (arm
    reference + last gripper command), expressed relative to the measured
    pose, so it leads the state.
    """

    VERSION = tfds.core.Version('1.0.0')
    RELEASE_NOTES = {'1.0.0': 'Osama tri_yolo scene, seeds replayed, one episode per pick.'}

    def _info(self) -> tfds.core.DatasetInfo:
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict({
                'steps': tfds.features.Dataset({
                    'observation': tfds.features.FeaturesDict({
                        'image_top': _image('top', 'Closest to the real Arducam; blind at grasp.'),
                        'image_right': _image('right', 'Closest to the real SVPRO.'),
                        'image_left': _image('left', 'Mirror of right.'),
                        'state': tfds.features.Tensor(
                            shape=(8,), dtype=np.float32,
                            doc='[x, y, z, qx, qy, qz, qw, gripper_open]: link6 pose in the '
                                'robot base frame (MoveIt FK of the measured joints) and gripper '
                                'opening in [0, 1], 1 = open.'),
                        'joint_state': tfds.features.Tensor(
                            shape=(7,), dtype=np.float32,
                            doc='Measured 6 arm joints + gripper_controller (rad).'),
                    }),
                    'action': tfds.features.Tensor(
                        shape=(7,), dtype=np.float32,
                        doc='[dx, dy, dz, droll, dpitch, dyaw, gripper_open]: from the measured '
                            'link6 pose to the commanded one (m; small-angle Euler xyz of '
                            'R_state^-1 R_command, rad); gripper = commanded opening, absolute.'),
                    'joint_action': tfds.features.Tensor(
                        shape=(7,), dtype=np.float32,
                        doc='Commanded 6 arm joints + gripper_controller (rad).'),
                    'discount': tfds.features.Scalar(dtype=np.float32, doc='1.'),
                    'reward': tfds.features.Scalar(dtype=np.float32, doc='1 on the final step.'),
                    'is_first': tfds.features.Scalar(dtype=np.bool_, doc='First step.'),
                    'is_last': tfds.features.Scalar(dtype=np.bool_, doc='Last step.'),
                    'is_terminal': tfds.features.Scalar(dtype=np.bool_, doc='Last step (demos).'),
                    'language_instruction': tfds.features.Text(
                        doc='"pick up the <object> and place it in the <colour> bin", one '
                            'phrasing per object, the vocabulary of the 60-episode dataset.'),
                }),
                'episode_metadata': tfds.features.FeaturesDict({
                    'file_path': tfds.features.Text(doc='Source .npy (seed, piece, attempt).'),
                }),
            }))

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        return {'train': self._generate_examples(path='data/train/episode_*.npy')}

    def _generate_examples(self, path) -> Iterator[Tuple[str, Any]]:
        def _parse(episode_path):
            data = np.load(episode_path, allow_pickle=True)
            steps = [{
                'observation': {k: s[k] for k in ('image_top', 'image_right', 'image_left',
                                                  'state', 'joint_state')},
                **{k: s[k] for k in ('action', 'joint_action', 'discount', 'reward', 'is_first',
                                     'is_last', 'is_terminal', 'language_instruction')},
            } for s in data]
            return episode_path, {'steps': steps,
                                  'episode_metadata': {'file_path': os.path.basename(episode_path)}}

        for episode_path in sorted(glob.glob(path)):
            yield _parse(episode_path)
