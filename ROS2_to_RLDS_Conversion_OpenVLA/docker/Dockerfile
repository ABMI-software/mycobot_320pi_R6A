# rlds_builder -- isolated env for moojink/rlds_dataset_builder.
# Deliberately separate from the gazebo_to_lerobot ROS2 container: this repo pins
# Python 3.9 + TensorFlow 2.13.0, which has nothing to do with ROS2/MoveIt2.
# The two containers talk to each other only through files (intermediate
# per-episode .npy produced inside gazebo_to_lerobot, consumed here).
#
# Plain pip into python:3.9-slim, NOT conda: a first attempt using
# `conda env create` on this ~4.8GB-RAM VM was killed by the host's
# low-memory guard -- conda's SAT solver loads full repodata into memory
# before installing anything, expensive even for a small, fully-pinned env.
# requirements*.txt below are extracted verbatim from the repo's own
# environment_ubuntu.yml `pip:` section (every version already pinned), so
# pip's resolver never backtracks. The 3 heaviest wheels (tensorflow,
# tensorflow-datasets, apache-beam) each get their own RUN layer so a build
# that dies partway leaves the earlier layers cached instead of losing
# everything on retry.
FROM python:3.9-slim

ENV PIP_NO_CACHE_DIR=1
WORKDIR /workspace

RUN pip install --upgrade pip==23.2.1

COPY docker/requirements_rest.txt /tmp/requirements_rest.txt
RUN pip install --no-deps -r /tmp/requirements_rest.txt

RUN pip install --no-deps tensorflow==2.13.0
RUN pip install --no-deps tensorflow-datasets==4.9.2
RUN pip install --no-deps apache-beam==2.49.0

CMD ["sleep", "infinity"]
