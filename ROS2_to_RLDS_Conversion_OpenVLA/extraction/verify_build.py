import tensorflow_datasets as tfds

ds = tfds.load("gazebo_to_lerobot_mycobot", split="train")
for i, ep in enumerate(ds):
    steps = list(ep["steps"])
    fp = ep["episode_metadata"]["file_path"].numpy()
    print("episode", i, ":", len(steps), "steps, file=", fp)
    print("  first image shape:", steps[0]["observation"]["image"].shape)
    print("  first state:", steps[0]["observation"]["state"].numpy())
    print("  instruction:", steps[0]["language_instruction"].numpy())
    print("  is_first/is_last (0):", steps[0]["is_first"].numpy(), steps[0]["is_last"].numpy())
    print("  is_first/is_last (-1):", steps[-1]["is_first"].numpy(), steps[-1]["is_last"].numpy())
