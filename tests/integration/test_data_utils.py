import logging
from pathlib import Path

import numpy as np
import pytest

from minialoha.utils.data_utils import create_dataset_path, save_to_hdf5

logger = logging.getLogger(__name__)


@pytest.fixture
def data_test_dir_path():
    current_directory = Path.cwd()
    dataset_dir = current_directory / "tests" / "test_data"
    return dataset_dir


@pytest.fixture
def data_dict():
    """Return a dictionary for testing save_to_hdf5."""
    data_dict = {
        "/observations/qpos": [
            np.array([3497.0, 1893.0, 1121.0, 903.0, 2058.0]),
            np.array([3485.0, 1886.0, 1121.0, 906.0, 2065.0]),
            np.array([3412.0, 1863.0, 1123.0, 942.0, 2085.0]),
        ],
        # "/observations/qvel": [],
        # "/observations/effort": [],
        "/action": [
            np.array([2762, 1818, 1127, 1073, 2120], dtype=np.int32),
            np.array([2762, 1818, 1127, 1073, 2120], dtype=np.int32),
            np.array([2762, 1818, 1127, 1073, 2120], dtype=np.int32),
        ],
    }
    return data_dict


def test_save_to_hdf5(data_dict, data_test_dir_path):
    """Test save_to_hdf5 function."""
    dataset_path = create_dataset_path(
        dataset_dir=data_test_dir_path,
        dataset_filename="test_save_to_hdf5.hdf5",
        overwrite=True,
    )

    camera_names = []
    max_timesteps = 3

    logger.info(f"dataset_path: {dataset_path}")
    # TODO: Manage deleting the file after testing
    save_to_hdf5(data_dict, dataset_path, camera_names, max_timesteps)
