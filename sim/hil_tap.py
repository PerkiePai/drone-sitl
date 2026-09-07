#!/usr/bin/env python3
# ============================================================================
# hil_tap.py -- a MITM tap on the PX4 <-> Isaac Sim HIL MAVLink link.
#
# Two jobs:
#   1. SHOW  the thrust stream PX4 sends into Isaac (HIL_ACTUATOR_CONTROLS --
#      one normalised value per rotor, 0.0..1.0) and the sensor stream coming
#      back (HIL_SENSOR / HIL_GPS).
#   2. FREEZE that stream on command, so you can watch what a stalled bridge
#      does to the sim. Under lockstep, withholding the PX4->Isaac direction
#      halts Isaac's physics step within ~1 frame; this tool measures how fast
#      the HIL_SENSOR return stream flatlines and prints it.
#
# Topology (PX4 always dials TCP 4560; we sit in the middle):
#
#     PX4  --tcp connect-->  hil_tap :4560  --tcp connect-->  Isaac :4561
#
# so Isaac must be told to listen on 4561 instead of 4560:
#
#     DRONE_SETUP_SIM_TCP_BASEPORT=4561 ./sim/run-website.sh
#     conda run -n drone python sim/hil_tap.py
#
# Controls (while running):
#     <Enter>   toggle freeze / thaw
#     s<Enter>  print a one-line status
#     q<Enter>  quit (also Ctrl-C)
#     kill -USR1 <pid>           toggle freeze from another shell
#     echo freeze > <ctl-file>   toggle via a plain file -- no PID needed.
#     echo thaw   > <ctl-file>   Path is --control-file (default
#                                 logs/hil_tap.ctl next to this script).
#
# Also writes logs/hil_tap.status.json once per --interval (--status-file) --
# a poller reads this instead of tailing the console log; the webpage's
# GET /hil/status does exactly that.
#
# This is a throwaway diagnostic. It re-encodes nothing -- bytes are copied
# through verbatim; MAVLink is parsed only on a copy, for the display.
# ============================================================================
import argparse
import json
import os
import signal
import socket
import sys
import threading
import time

from pymavlink.dialects.v20 import common as mav

ACT = mav.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS   # 93  PX4 -> Isaac
SENS = mav.MAVLINK_MSG_ID_HIL_SENSOR             # 107 Isaac -> PX4
GPS = mav.MAVLINK_MSG_ID_HIL_GPS                 # 113 Isaac -> PX4


class Counter:
    """Exact message count, plus a per-window count for the display line.

    No smoothing -- what you see is what actually crossed the wire in the
    last window, not an inter-arrival-gap estimate that can go stale.
    """

    def __init__(self):
        self.count = 0          # lifetime total (halt-detection reads this)
        self._window = 0        # since the last take_window()

    def tick(self):
        self.count += 1
        self._window += 1

    def take_window(self):
        """Return the count since the last call, and reset it."""
        n, self._window = self._window, 0
        return n


class Tap:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.frozen = threading.Event()

        self.px4_sock = None
        self.isaac_sock = None

        # display state
        self.last_controls = None          # list[float]
        self.last_sim_t = 0.0              # seconds, from newest HIL msg time_usec
        self.act_count = Counter()
        self.sens_count = Counter()
        self.gps_count = Counter()

        # freeze bookkeeping
        self._held = bytearray()           # PX4->Isaac bytes withheld while frozen
        self._held_lock = threading.Lock() # _pump_p2i (writer) vs toggle() (flusher)
        self._freeze_wall = 0.0
        self._i2p_at_freeze = 0            # HIL_SENSOR count at the moment we froze
        self._last_i2p_wall = 0.0          # wall time of last byte seen Isaac->PX4
        self._halt_reported = False

        # control-file freeze toggle (the "button" -- write freeze/thaw to it)
        self._ctl_seen = None              # last content acted on, so a repeat write is a no-op

    # -- connection setup ---------------------------------------------------
    def _connect_isaac(self):
        host, port = self.args.isaac_host, self.args.isaac_port
        deadline = time.time() + self.args.connect_timeout
        while time.time() < deadline and not self.stop.is_set():
            try:
                s = socket.create_connection((host, port), timeout=3)
                # create_connection's `timeout` sticks on the socket after
                # connect (it does not reset to blocking). Left in place, a
                # freeze longer than 3s makes the next recv() in _pump_i2p
                # raise socket.timeout -- an OSError subclass -- which our
                # `except OSError: break` reads as "peer closed" and tears
                # the whole tap down, closing BOTH sides of the link. That
                # is almost certainly what wedged the real sim during the
                # first long freeze this session. recv() must block here.
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"[tap] connected to Isaac at {host}:{port}")
                return s
            except OSError:
                time.sleep(0.5)
        raise SystemExit(f"[tap] could not reach Isaac at {host}:{port} "
                         f"in {self.args.connect_timeout}s -- is it up with "
                         f"DRONE_SETUP_SIM_TCP_BASEPORT={port}?")

    def _accept_px4(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.args.listen_host, self.args.listen_port))
        srv.listen(1)
        srv.settimeout(1.0)
        print(f"[tap] listening for PX4 on {self.args.listen_host}:{self.args.listen_port}")
        while not self.stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            conn.settimeout(None)   # belt-and-braces: must block, see _connect_isaac
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[tap] PX4 connected from {addr[0]}:{addr[1]}")
            srv.close()
            return conn
        srv.close()
        raise SystemExit("[tap] stopped before PX4 connected")

    # -- pumps ------------------------------------------------------------
    def _pump_p2i(self):
        """PX4 -> Isaac. This is the direction we withhold when frozen."""
        parser = mav.MAVLink(None)
        parser.robust_parsing = True
        src, dst = self.px4_sock, self.isaac_sock
        while not self.stop.is_set():
            try:
                data = src.recv(4096)
            except OSError:
                break
            if not data:
                break
            self._scan(parser, data, outbound=True)
            if self.frozen.is_set():
                with self._held_lock:
                    self._held.extend(data)
                continue
            try:
                dst.sendall(data)
            except OSError:
                break
        self._shutdown("PX4->Isaac closed")

    def _pump_i2p(self):
        """Isaac -> PX4. Always forwarded; when Isaac halts there is simply
        nothing here to forward, which is exactly what we measure."""
        parser = mav.MAVLink(None)
        parser.robust_parsing = True
        src, dst = self.isaac_sock, self.px4_sock
        while not self.stop.is_set():
            try:
                data = src.recv(4096)
            except OSError:
                break
            if not data:
                break
            self._last_i2p_wall = time.time()
            self._scan(parser, data, outbound=False)
            try:
                dst.sendall(data)
            except OSError:
                break
        self._shutdown("Isaac->PX4 closed")

    # -- MAVLink peek (on a copy; never re-encoded) -----------------------
    def _scan(self, parser, data, outbound):
        try:
            msgs = parser.parse_buffer(data) or []
        except (mav.MAVError, ValueError):
            return
        for m in msgs:
            mid = m.get_msgId()
            if mid == ACT:
                self.last_controls = list(m.controls[:self.args.rotors])
                self.last_sim_t = m.time_usec / 1e6
                self.act_count.tick()
            elif mid == SENS:
                self.last_sim_t = m.time_usec / 1e6
                self.sens_count.tick()
            elif mid == GPS:
                self.gps_count.tick()

    # -- freeze / thaw ---------------------------------------------------
    def toggle(self, auto=False):
        if self.frozen.is_set():
            # Flush BEFORE clearing the flag: PX4 is blocked waiting for the
            # sensor reply to whatever is in _held, so nothing will arrive on
            # px4_sock to trigger a flush from _pump_p2i's own loop -- that
            # was a deadlock (thaw printed, but the stream never actually
            # resumed). Sending it directly here is what actually unsticks it.
            with self._held_lock:
                held = bytes(self._held)
                self._held.clear()
            sent_ok = True
            if held:
                try:
                    self.isaac_sock.sendall(held)
                except OSError as e:
                    sent_ok = False
                    print(f"[tap] THAW flush failed: {e!r}")
            self.frozen.clear()
            tag = "AUTO-THAW" if auto else "THAW"
            status = f"releasing {len(held)} held bytes" if sent_ok else "FLUSH FAILED"
            print(f"\n[tap] {tag}  -- {status}, sim resumes")
        else:
            self._freeze_wall = time.time()
            self._i2p_at_freeze = self.sens_count.count
            self._halt_reported = False
            self.frozen.set()
            print(f"\n[tap] FREEZE -- withholding PX4->Isaac thrust. "
                  f"watching HIL_SENSOR return stream "
                  f"(auto-thaw in {self.args.max_freeze:.0f}s)...")

    def _watch_halt(self):
        while not self.stop.is_set():
            time.sleep(0.05)
            if not self.frozen.is_set():
                continue
            since_freeze = time.time() - self._freeze_wall
            quiet = time.time() - self._last_i2p_wall
            extra = self.sens_count.count - self._i2p_at_freeze
            if not self._halt_reported and quiet > 0.15 and since_freeze > 0.1:
                lag_ms = max(0.0, (self._last_i2p_wall - self._freeze_wall) * 1e3)
                print(f"[tap] Isaac HALTED: HIL_SENSOR return stream went "
                      f"silent {quiet*1e3:.0f} ms ago "
                      f"({extra} sensor msg(s) got through in the "
                      f"{lag_ms:.0f} ms after freeze, then nothing). "
                      f"sim clock stuck at t={self.last_sim_t:.3f}s")
                self._halt_reported = True
            # Safety: a freeze longer than this makes PX4's simulator_mavlink
            # give up on the link for good (poll-timeout spam, no reconnect,
            # orphaned process -> full restart). Thaw ourselves before that.
            if self.args.max_freeze > 0 and since_freeze > self.args.max_freeze:
                print(f"[tap] hit {self.args.max_freeze:.0f}s freeze limit")
                self.toggle(auto=True)

    # -- display -------------------------------------------------------
    def _display(self):
        """Print exact message counts seen in each window -- not a smoothed
        rate, the literal number that crossed the wire since the last line."""
        period = self.args.interval
        while not self.stop.is_set():
            time.sleep(period)
            state = "FROZEN" if self.frozen.is_set() else "  live"
            if self.last_controls is None:
                thr = "m=[--]"
            else:
                thr = "m=[" + " ".join(f"{c:.2f}" for c in self.last_controls) + "]"
            acts = self.act_count.take_window()
            sens = self.sens_count.take_window()
            gps = self.gps_count.take_window()
            line = (f"[{state}] t={self.last_sim_t:8.3f}s  "
                    f"P->I {thr} acts={acts:4d}/{period:g}s   "
                    f"I->P sensor={sens:4d}/{period:g}s "
                    f"gps={gps:4d}/{period:g}s")
            print(line)
            self._write_status(acts, sens, gps)

    def _write_status(self, acts, sens, gps):
        """One JSON snapshot per display tick, for anything polling this tap
        without tailing the console log -- currently the webpage's /hil/status.
        Atomic write (tmp + rename) so a reader never sees a half-written
        file; a stale mtime is how a reader tells the tap is not running."""
        path = self.args.status_file
        if not path:
            return
        snapshot = {
            "ts": time.time(),
            "frozen": self.frozen.is_set(),
            "sim_t": self.last_sim_t,
            "controls": self.last_controls,
            "interval_s": self.args.interval,
            "acts_per_interval": acts,
            "sensor_per_interval": sens,
            "gps_per_interval": gps,
        }
        tmp = path + ".tmp"
        try:
            with open(tmp, "w") as fh:
                json.dump(snapshot, fh)
            os.replace(tmp, path)
        except OSError:
            pass

    # -- control-file "button" ----------------------------------------
    def _control_file(self):
        """Poll a plain text file for freeze/thaw commands -- lets anything
        (a shell, another script, you) toggle without hunting for a PID:
            echo freeze > logs/hil_tap.ctl
            echo thaw   > logs/hil_tap.ctl
        Acts only on a change from the last content seen, so leaving the
        file as-is (or writing the same word twice) does not re-toggle."""
        path = self.args.control_file
        if not path:
            return
        while not self.stop.is_set():
            time.sleep(0.2)
            try:
                content = open(path).read().strip().lower()
            except OSError:
                continue
            if not content or content == self._ctl_seen:
                continue
            self._ctl_seen = content
            if content in ("freeze", "1", "on") and not self.frozen.is_set():
                self.toggle()
            elif content in ("thaw", "0", "off") and self.frozen.is_set():
                self.toggle()
            elif content == "toggle":
                self.toggle()

    # -- stdin control -----------------------------------------------
    def _stdin(self):
        for raw in sys.stdin:
            cmd = raw.strip().lower()
            if cmd in ("q", "quit", "exit"):
                self.stop.set()
                break
            if cmd == "s":
                print(f"[tap] frozen={self.frozen.is_set()} "
                      f"held={len(self._held)}B "
                      f"acts={self.act_count.count} sensors={self.sens_count.count}")
                continue
            self.toggle()

    def _shutdown(self, why):
        if not self.stop.is_set():
            print(f"\n[tap] shutting down -- {why}")
        self.stop.set()

    # -- run ---------------------------------------------------------
    def run(self):
        signal.signal(signal.SIGUSR1, lambda *_: self.toggle())
        signal.signal(signal.SIGINT, lambda *_: self._shutdown("Ctrl-C"))
        signal.signal(signal.SIGTERM, lambda *_: self._shutdown("SIGTERM"))

        self.isaac_sock = self._connect_isaac()
        self.px4_sock = self._accept_px4()
        self._last_i2p_wall = time.time()

        # Leftover content from a previous run must not fire on startup.
        if self.args.control_file:
            try:
                self._ctl_seen = open(self.args.control_file).read().strip().lower()
            except OSError:
                pass

        threads = [
            threading.Thread(target=self._pump_p2i, name="p2i", daemon=True),
            threading.Thread(target=self._pump_i2p, name="i2p", daemon=True),
            threading.Thread(target=self._watch_halt, name="halt", daemon=True),
            threading.Thread(target=self._display, name="disp", daemon=True),
            threading.Thread(target=self._stdin, name="stdin", daemon=True),
            threading.Thread(target=self._control_file, name="ctl", daemon=True),
        ]
        for t in threads:
            t.start()

        ctl_hint = f"  ctl-file={self.args.control_file}" if self.args.control_file else ""
        print(f"[tap] up. pid {os.getpid()}. <Enter>=freeze/thaw  s=status  q=quit{ctl_hint}")
        try:
            while not self.stop.is_set():
                time.sleep(0.2)
        finally:
            self.stop.set()
            for s in (self.px4_sock, self.isaac_sock):
                try:
                    s and s.close()
                except OSError:
                    pass
        print("[tap] done")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=4560,
                   help="port PX4 dials (default 4560)")
    p.add_argument("--isaac-host", default="127.0.0.1")
    p.add_argument("--isaac-port", type=int, default=4561,
                   help="port Isaac listens on -- set DRONE_SETUP_SIM_TCP_BASEPORT to match")
    p.add_argument("--rotors", type=int, default=4)
    p.add_argument("--interval", type=float, default=1.0,
                   help="seconds per status line; each line shows the exact "
                        "message count seen in that window (default: 1s)")
    p.add_argument("--max-freeze", type=float, default=20.0,
                   help="auto-thaw after this many seconds (0 = never; "
                        "a longer freeze permanently wedges PX4's sim link)")
    p.add_argument("--control-file",
                   default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "logs", "hil_tap.ctl"),
                   help="write freeze/thaw to this file to toggle without a "
                        "PID (empty string disables)")
    p.add_argument("--status-file",
                   default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "logs", "hil_tap.status.json"),
                   help="JSON snapshot written once per --interval, for a "
                        "poller (e.g. the webpage's /hil/status) that does "
                        "not want to tail the console log (empty disables)")
    p.add_argument("--connect-timeout", type=float, default=120.0)
    Tap(p.parse_args()).run()


if __name__ == "__main__":
    main()
