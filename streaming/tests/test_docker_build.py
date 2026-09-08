"""streaming.docker_build.DockerBuild -- builds an uploaded bundle.

No real Docker in these tests -- asyncio.create_subprocess_exec is
monkeypatched. Task 5's manual pass is what exercises a real Docker
daemon end to end; pytest-asyncio is not a dependency of this repo (see
streaming/tests/test_web_ui.py's existing `async def exercise():` +
`asyncio.run(exercise())` pattern), so every test here follows that same
shape: a plain `def test_...():` that wraps its async body in
`asyncio.run(...)`, not `@pytest.mark.asyncio`.
"""
import asyncio
import io
import os
import sys
import tarfile

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "streaming"))

import docker_build  # noqa: E402


def _make_bundle(tmp_path, with_dockerfile=True):
    src = tmp_path / "src"
    src.mkdir()
    if with_dockerfile:
        (src / "Dockerfile").write_text("FROM busybox\n")
    tar_path = tmp_path / "bundle.tar"
    with tarfile.open(tar_path, "w") as tar:
        for f in src.iterdir():
            tar.add(f, arcname=f.name)
    return str(tar_path)


class FakeProc:
    """Stands in for asyncio.subprocess.Process."""
    def __init__(self, lines, returncode):
        self._lines = [l.encode() + b"\n" for l in lines]
        self.returncode = returncode
        self.stdout = self

    def __aiter__(self):
        return self

    async def __anext__(self):
        if not self._lines:
            raise StopAsyncIteration
        return self._lines.pop(0)

    async def wait(self):
        return self.returncode


def test_build_succeeds_and_sets_state_built(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        assert args[0] == "docker" and args[1] == "build"
        return FakeProc(["Step 1/1 : FROM busybox", "Successfully built abc123"], 0)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is True
        assert image_tag.startswith("submission-myagent-")
        assert db.state == "built"
        assert db.image_tag == image_tag
        assert "Successfully built abc123" in db.snapshot()["log"]

    asyncio.run(run())


def test_build_failure_sets_state_error(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        return FakeProc(["ERROR: something broke"], 1)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"
        assert any("exited 1" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_missing_dockerfile_fails_before_any_subprocess_call(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path, with_dockerfile=False)
    called = []

    async def fake_exec(*args, **kwargs):
        called.append(args)
        raise AssertionError("should never reach docker build")

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert image_tag is None
        assert db.state == "error"
        assert not called
        assert any("no Dockerfile" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_docker_not_installed_sets_state_error(tmp_path, monkeypatch):
    tar_path = _make_bundle(tmp_path)

    async def fake_exec(*args, **kwargs):
        raise FileNotFoundError("docker")

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        db = docker_build.DockerBuild()
        ok, image_tag = await db.build(tar_path, "myagent")
        assert ok is False
        assert db.state == "error"
        assert any("not found" in line for line in db.snapshot()["log"])

    asyncio.run(run())


def test_tar_member_escaping_the_context_is_rejected(tmp_path):
    """A malicious bundle with `../../etc/passwd`-style paths must not
    write outside the extraction directory."""
    tar_path = tmp_path / "evil.tar"
    with tarfile.open(tar_path, "w") as tar:
        info = tarfile.TarInfo(name="../../evil.txt")
        info.size = 4
        tar.addfile(info, io.BytesIO(b"evil"))

    dest = tmp_path / "dest"
    dest.mkdir()
    with tarfile.open(tar_path) as tar:
        with pytest.raises(ValueError, match="escapes"):
            docker_build._safe_extract(tar, str(dest))


def test_list_images_parses_docker_output(monkeypatch):
    async def fake_exec(*args, **kwargs):
        assert args[:3] == ("docker", "image", "ls")
        class P:
            async def communicate(self):
                return (b"submission-a-1:latest\t2026-09-08 10:00:00\n"
                        b"submission-b-2:latest\t2026-09-08 11:00:00\n", b"")
        return P()

    monkeypatch.setattr(asyncio, "create_subprocess_exec", fake_exec)

    async def run():
        images = await docker_build.list_images()
        assert images[0]["tag"] == "submission-b-2:latest"    # newest first
        assert images[1]["tag"] == "submission-a-1:latest"

    asyncio.run(run())
