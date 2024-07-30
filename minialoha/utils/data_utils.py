import os
import time
from typing import Any, Dict, List

import h5py


def create_dataset_path(dataset_dir, dataset_filename: str, overwrite: bool) -> str:
    """Create the path in the filesystem for the dataset."""
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_filename)
    if os.path.isfile(dataset_path) and not overwrite:
        print(
            f"Dataset already exist at \n{dataset_path}\nHint: set overwrite to True."
        )
        raise SystemExit()
    return dataset_path


def save_to_hdf5(
    data_dict: Dict[str, Any],
    dataset_path: str,
    camera_names: List[str],
    max_timesteps: int,
):
    """Save data_dict to HDF5 file at dataset_path."""
    # HDF5
    t0 = time.time()
    with h5py.File(dataset_path, "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        obs = root.create_group("observations")
        image = obs.create_group("images")
        for cam_name in camera_names:
            _ = image.create_dataset(
                cam_name,
                (max_timesteps, 480, 640, 3),
                dtype="uint8",
                chunks=(1, 480, 640, 3),
            )
            # compression='gzip',compression_opts=2,)
            # compression=32001, compression_opts=(0, 0, 0, 0, 9, 1, 1), shuffle=False)
        _ = obs.create_dataset("qpos", (max_timesteps, 14))
        _ = obs.create_dataset("qvel", (max_timesteps, 14))
        _ = obs.create_dataset("effort", (max_timesteps, 14))
        _ = root.create_dataset("action", (max_timesteps, 14))

        for name, array in data_dict.items():
            dataset = root[name]
            dataset[...] = array
    print(f"Saving: {time.time() - t0:.1f} secs")
