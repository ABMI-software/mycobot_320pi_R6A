#!/usr/bin/env python3
"""Run a command as a child subreaper, so the processes it orphans are reaped.

  reap_children.py <command> [args...]      # e.g. reap_children.py record_tri_batch.sh 1051 1400

gazebo_to_lerobot's PID 1 is `sleep infinity`, which never reaps: every process a seed's
teardown orphans (Gazebo bridges, the scene randomizer, ground truth, ...) stayed a zombie,
about 10 per seed (1052 after batch B 1001-1050 and the tests, cleared by restarting the
container on 19/10). Under this wrapper they are reparented here instead and reaped as they
exit. Once the command ends, it keeps reaping until no descendant is left (one still alive
then would otherwise end up under PID 1 again), and exits with the command's status.
Plain python3, Linux only.
"""
import ctypes
import os
import sys

PR_SET_CHILD_SUBREAPER = 36


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        sys.exit(f'prctl(PR_SET_CHILD_SUBREAPER) failed: {os.strerror(ctypes.get_errno())}')
    child = os.fork()
    if child == 0:
        os.execvp(sys.argv[1], sys.argv[1:])
    code = None
    while True:
        try:
            pid, status = os.waitpid(-1, 0)
        except ChildProcessError:
            sys.exit(code)
        if pid == child:
            code = os.waitstatus_to_exitcode(status) & 0xFF


if __name__ == '__main__':
    main()
