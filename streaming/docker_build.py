"""Builds a competitor's uploaded Docker bundle. Mirrors AgentRun's shape
(state machine + capped log) but for `docker build` instead of flying a
script. See
docs/superpowers/specs/2026-09-08-docker-submission-website-integration-design.md.
"""
import asyncio
import collections
import os
import shutil
import tarfile
import tempfile
import time

IMAGE_PREFIX = "submission-"


def _safe_extract(tar, dest):
    """Extract `tar` into `dest`, refusing any member whose resolved path
    would land outside `dest` (path traversal via `../` or an absolute
    path). Python 3.11's tarfile.extractall has no `filter=` kwarg
    (that's 3.12+), so this is done by hand rather than relying on one."""
    dest_real = os.path.realpath(dest)
    for member in tar.getmembers():
        member_path = os.path.realpath(os.path.join(dest, member.name))
        if member_path != dest_real and not member_path.startswith(dest_real + os.sep):
            raise ValueError(f"tar member escapes the build context: {member.name!r}")
    tar.extractall(dest)


class DockerBuild:
    def __init__(self, base_image="competition-base:latest"):
        self.base_image = base_image
        self.state = "idle"          # idle | building | built | error
        self.image_tag = None
        self._log = collections.deque(maxlen=40)

    def snapshot(self):
        return {"state": self.state, "image_tag": self.image_tag,
                "log": list(self._log)}

    def _note(self, line):
        self._log.append(line)

    async def build(self, tar_path, stem):
        """Extract `tar_path`, docker build it, tag submission-<stem>-<ts>.
        Returns (ok, image_tag | None)."""
        self.state = "building"
        self._log.clear()
        ts = time.strftime("%Y%m%d-%H%M%S")
        image_tag = f"{IMAGE_PREFIX}{stem}-{ts}"
        ctx = tempfile.mkdtemp(prefix="docker-build-")
        try:
            with tarfile.open(tar_path) as tar:
                _safe_extract(tar, ctx)
            dockerfile = os.path.join(ctx, "Dockerfile")
            if not os.path.isfile(dockerfile):
                self.state = "error"
                self._note("no Dockerfile at the bundle's root")
                return False, None

            self._note(f"building {image_tag} ...")
            proc = await asyncio.create_subprocess_exec(
                "docker", "build", "-t", image_tag, "-f", dockerfile, ctx,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT)
            async for line in proc.stdout:
                self._note(line.decode(errors="replace").rstrip())
            code = await proc.wait()

            if code != 0:
                self.state = "error"
                self._note(f"docker build exited {code}")
                return False, None

            self.state = "built"
            self.image_tag = image_tag
            self._note(f"built {image_tag}")
            return True, image_tag
        except FileNotFoundError:
            self.state = "error"
            self._note("docker not found -- is it installed on this host?")
            return False, None
        except Exception as exc:                      # noqa: BLE001
            self.state = "error"
            self._note(f"build failed: {exc!r}")
            return False, None
        finally:
            shutil.rmtree(ctx, ignore_errors=True)


async def list_images():
    """Built submission images, newest first: [{"tag":..., "created_at":...}]."""
    proc = await asyncio.create_subprocess_exec(
        "docker", "image", "ls",
        "--filter", f"reference={IMAGE_PREFIX}*",
        "--format", "{{.Repository}}:{{.Tag}}\t{{.CreatedAt}}",
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
    out, _ = await proc.communicate()
    rows = []
    for line in out.decode(errors="replace").splitlines():
        if not line.strip():
            continue
        tag, _, created = line.partition("\t")
        rows.append({"tag": tag, "created_at": created})
    rows.sort(key=lambda r: r["created_at"], reverse=True)
    return rows
