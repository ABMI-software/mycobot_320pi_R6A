#!/usr/bin/env python3
"""Record each pick-and-place of Osama's sort as one rosetta episode.

  python3 record_tri_sort.py --seed 1 --out /workspace/tri_sort/seed_001

Listens to sim_sorting_grasp's /pickplace/status, which it does not modify:
"▶ <piece> : saisie ..." starts a RecordEpisode goal with that piece's
instruction as the prompt; "<piece> (essai N) : <verdict>" stops it. One JSON
per episode (ep_<piece>_aN.json) carries the verdict and the bag path; it is
metadata, never part of the recorded observation (spec A5). Exits once
sim_sorting_grasp has left the graph.
"""
import argparse
import json
import re
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosetta_interfaces.action import RecordEpisode
from std_msgs.msg import String

# Same wording as the 60-episode dataset (config/tasks.jsonl), so the two
# datasets share one instruction vocabulary.
PIECES = {
    'cube_rouge': ('bac_rouge', 'pick up the red cube and place it in the red bin'),
    'cube_bleu': ('bac_bleu', 'pick up the blue cube and place it in the blue bin'),
    'cylindre_vert': ('bac_vert', 'pick up the green cylinder and place it in the green bin'),
    'pave_jaune': ('bac_jaune', 'pick up the yellow box and place it in the yellow bin'),
}
START = re.compile(r'▶ (\w+) : saisie')
VERDICT = re.compile(r'^\s*(\w+) \(essai (\d+)\) : (.*)$')


class Recorder(Node):
    def __init__(self, seed, out, max_duration_s):
        super().__init__('record_tri_sort',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.seed, self.out, self.max_duration_s = seed, out, max_duration_s
        self.client = ActionClient(self, RecordEpisode, 'record_episode')
        self.goal_handle, self.piece, self.started = None, None, None
        self.episodes = []
        # Handled from the main loop: begin/end wait on action futures, which
        # cannot be spun from inside a subscription callback.
        self.pending = []
        self.create_subscription(String, '/pickplace/status',
                                 lambda msg: self.pending.append(msg.data), 10)

    def handle_status(self, text):
        start, verdict = START.search(text), VERDICT.match(text)
        if start and start.group(1) in PIECES:
            self.begin(start.group(1))
        elif verdict and verdict.group(1) == self.piece:
            self.end(int(verdict.group(2)), verdict.group(3))

    def begin(self, piece):
        if self.goal_handle is not None:
            self.get_logger().error(f'{piece} started while {self.piece} still recording')
            return
        goal = RecordEpisode.Goal(prompt=PIECES[piece][1], max_duration_s=self.max_duration_s)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'record_episode goal rejected for {piece}')
            return
        self.goal_handle, self.piece, self.started = handle, piece, self.get_clock().now()
        self.get_logger().info(f'recording {piece}: {goal.prompt!r}')

    def end(self, attempt, verdict):
        cancel = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=30.0)
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        result = result_future.result().result
        bin_, instruction = PIECES[self.piece]
        meta = {
            'seed': self.seed, 'piece': self.piece, 'bin': bin_, 'instruction': instruction,
            'attempt': attempt, 'verdict': verdict, 'ok': verdict.startswith('OK'),
            'bag_path': result.bag_path, 'messages_written': result.messages_written,
            'termination_reason': result.termination_reason,
            'sim_seconds': (self.get_clock().now() - self.started).nanoseconds / 1e9,
            'pose_source': 'ground_truth', 'scene': 'tri_yolo',
        }
        path = self.out / f'ep_{self.piece}_a{attempt}.json'
        path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n')
        self.episodes.append(meta)
        self.get_logger().info(f'{self.piece} stopped: {verdict} -- {result.messages_written} '
                               f'messages, {result.bag_path}')
        self.goal_handle, self.piece = None, None

    def sorter_running(self):
        return any(name == 'sim_sorting_grasp' for name, _ in self.get_node_names_and_namespaces())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', type=Path, required=True)
    # The recorder times this on the wall clock; at RTF ~0.05 one pick-and-place
    # takes 4-7 wall minutes. A guard, not the episode end.
    ap.add_argument('--max-duration-s', type=float, default=3600.0)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Recorder(args.seed, args.out, args.max_duration_s)
    if not node.client.wait_for_server(timeout_sec=120.0):
        raise SystemExit('record_episode action server not available -- recorder not active')
    seen_sorter = False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
        while node.pending:
            node.handle_status(node.pending.pop(0))
        running = node.sorter_running()
        seen_sorter |= running
        if seen_sorter and not running and node.goal_handle is None:
            break
    node.get_logger().info(f'{len(node.episodes)} episodes written to {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
