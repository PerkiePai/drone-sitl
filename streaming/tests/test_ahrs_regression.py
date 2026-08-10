"""The MahonyState refactor must not move batch AHRS output at all."""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
FIXTURE = os.path.join(FIXTURES, "ahrs_baseline.npy")
DATASET_NAME = os.path.join(FIXTURES, "ahrs_baseline_dataset.txt")


def baseline_dataset():
    """The dataset the baseline was captured from, or None if it is gone.

    Pinned by name rather than resolved as 'newest under ~/vio_dataset': a
    baseline is only meaningful against the exact recording it came from, and
    every new recording would otherwise silently invalidate it.
    """
    if not os.path.exists(DATASET_NAME):
        return None
    with open(DATASET_NAME) as fh:
        name = fh.read().strip()
    d = os.path.expanduser(os.path.join("~/vio_dataset", name))
    return d if os.path.isdir(d) else None


@pytest.mark.skipif(not os.path.exists(FIXTURE),
                    reason="no baseline fixture; see plan Task 1 Step 1.3")
def test_batch_output_matches_pre_refactor_baseline():
    from flow_odometry import load_dataset, compute_ahrs_attitude
    d = baseline_dataset()
    if d is None:
        pytest.skip(f"baseline dataset named in {DATASET_NAME} is not present")
    _, _, recs = load_dataset(d)
    got = np.array(compute_ahrs_attitude(d, recs[:400]))
    assert np.allclose(got, np.load(FIXTURE), atol=1e-12)
