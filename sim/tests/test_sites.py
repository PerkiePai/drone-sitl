"""Site config is the only part of the stage build that runs outside Kit, so it
is the only part that can carry real tests. They guard the things that actually
broke: a stale model path, a spawn altitude that ignores the ground plane, and a
secret pasted into tracked config."""
import dataclasses
import os
import re
import sys
from pathlib import Path

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import sites  # noqa: E402


def test_bangkok_site_is_registered():
    site = sites.get_site("bangkok-survey-040")
    assert site.latitude == pytest.approx(13.66156872)
    assert site.longitude == pytest.approx(100.298235)
    assert site.height == 0.0


def test_unknown_site_names_the_known_ones():
    with pytest.raises(KeyError, match="bangkok-survey-040"):
        sites.get_site("nowhere")


def test_every_model_path_exists():
    for site in sites.SITES.values():
        for model in site.models:
            path = model.resolved_path()
            assert path.is_file(), f"{site.name}/{model.prim_name}: missing {path}"


def test_spawn_sits_just_above_the_ground_plane():
    for site in sites.SITES.values():
        assert 0.0 < site.spawn_agl_m <= 2.0
        assert site.spawn_z == pytest.approx(site.ground_z + site.spawn_agl_m)


def test_config_carries_no_secrets():
    text = Path(sites.__file__).read_text(encoding="utf-8")
    assert not re.search(r"eyJ[A-Za-z0-9_-]{20,}", text), "JWT-shaped literal in tracked config"


def test_site_is_frozen():
    site = sites.get_site("bangkok-survey-040")
    with pytest.raises(dataclasses.FrozenInstanceError):
        site.latitude = 0.0
