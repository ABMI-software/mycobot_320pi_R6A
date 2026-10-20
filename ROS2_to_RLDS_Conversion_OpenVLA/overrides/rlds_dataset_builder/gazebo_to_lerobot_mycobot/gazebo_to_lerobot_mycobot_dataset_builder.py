from typing import Iterator, Tuple, Any

import glob
import numpy as np
import tensorflow_datasets as tfds


class GazeboToLerobotMycobot(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for the Gazebo_to_LeRobot_Pipeline MyCobot 320 Pi sim sandbox.

    Source: 2 rosetta-recorded episodes of scripted keyboard teleop in the
    Gazebo_to_LeRobot_Pipeline Gazebo Harmonic sandbox (no object, no real task -- pipeline
    proof only, see language_instruction below). Converted from ROS2 mcap
    bags by extraction/extract_episodes.py: topics extracted, resampled
    onto the camera's native ~10Hz sim-time grid (never the reverse -- you
    can interpolate joint angles, not images), end-effector pose computed
    per-timestep via MoveIt2 RobotState FK (cross-checked against the
    bag's own /tf chain, matched to 7 decimal places), position/rotation
    deltas computed between consecutive poses, gripper channel is the
    gripper_controller joint's absolute position (not a delta, matching
    the bridge_oxe convention: world_vector + rotation_delta + gripper).

    Deliberately NOT computing `language_embedding` (unlike example_dataset):
    that requires downloading and running a Universal Sentence Encoder from
    TF-Hub, real memory pressure on this box, and OpenVLA's own OXE
    transform functions only ever consume `language_instruction` text, not
    a precomputed embedding -- so it isn't load-bearing for OpenVLA and
    was skipped to keep this build cheap. Add it back (see example_dataset
    for the exact 3 lines) if a downstream consumer needs it.
    """

    VERSION = tfds.core.Version('1.0.0')
    RELEASE_NOTES = {
        '1.0.0': 'Initial release -- 2 pipeline-proof episodes, no real task.',
    }

    def _info(self) -> tfds.core.DatasetInfo:
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict({
                'steps': tfds.features.Dataset({
                    'observation': tfds.features.FeaturesDict({
                        'image': tfds.features.Image(
                            shape=(224, 224, 3),
                            dtype=np.uint8,
                            encoding_format='png',
                            doc='Front camera RGB, from /synth_camera/image/compressed '
                                '(resized+center-cropped from 320x240, BGR->RGB corrected).',
                        ),
                        'state': tfds.features.Tensor(
                            shape=(8,),
                            dtype=np.float32,
                            doc='[x, y, z, qx, qy, qz, qw, gripper] end-effector pose '
                                '(base_link frame, MoveIt2 FK) + gripper_controller joint '
                                'position, interpolated onto this camera frame\'s timestamp.',
                        ),
                    }),
                    'action': tfds.features.Tensor(
                        shape=(7,),
                        dtype=np.float32,
                        doc='[dx, dy, dz, droll, dpitch, dyaw, gripper]: position delta (m) '
                            'and small-angle Euler-xyz rotation delta (rad) to the next step\'s '
                            'pose; gripper is this step\'s absolute joint position, not a delta. '
                            'Zero pose delta on the final step of each episode.',
                    ),
                    'discount': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Discount if provided, default to 1.'
                    ),
                    'reward': tfds.features.Scalar(
                        dtype=np.float32,
                        doc='Reward if provided, 1 on final step for demos.'
                    ),
                    'is_first': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on first step of the episode.'
                    ),
                    'is_last': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode.'
                    ),
                    'is_terminal': tfds.features.Scalar(
                        dtype=np.bool_,
                        doc='True on last step of the episode if it is a terminal step, True for demos.'
                    ),
                    'language_instruction': tfds.features.Text(
                        doc='Honest, not aspirational: these 2 episodes are scripted keyboard '
                            'motion with no object and no goal, so the instruction describes '
                            'that plainly rather than naming a task that was never performed.'
                    ),
                }),
                'episode_metadata': tfds.features.FeaturesDict({
                    'file_path': tfds.features.Text(
                        doc='Path to the original data file.'
                    ),
                }),
            }))

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        """Both episodes go to 'train' -- 2 episodes is too few to carve out
        a meaningful val split, and pretending otherwise would misrepresent
        what this dataset actually is (a pipeline smoke test, not a
        trainable dataset)."""
        return {
            'train': self._generate_examples(path='data/train/episode_*.npy'),
        }

    def _generate_examples(self, path) -> Iterator[Tuple[str, Any]]:
        """Generator of examples for each split."""

        def _parse_example(episode_path):
            data = np.load(episode_path, allow_pickle=True)

            episode = []
            for step in data:
                episode.append({
                    'observation': {
                        'image': step['image'],
                        'state': step['state'],
                    },
                    'action': step['action'],
                    'discount': step['discount'],
                    'reward': step['reward'],
                    'is_first': step['is_first'],
                    'is_last': step['is_last'],
                    'is_terminal': step['is_terminal'],
                    'language_instruction': step['language_instruction'],
                })

            sample = {
                'steps': episode,
                'episode_metadata': {
                    'file_path': episode_path
                }
            }
            return episode_path, sample

        episode_paths = glob.glob(path)
        for sample in episode_paths:
            yield _parse_example(sample)
