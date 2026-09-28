"""Web UI for the CubeMars AKE60-8: stdlib HTTP server + 200 Hz control thread.

Run:  .venv/bin/python app.py            then open http://127.0.0.1:8060
      .venv/bin/python app.py --id 101 --channel /dev/cu.usbmodem101 --port 8060

The browser only sends setpoints; the control loop, limits and safety checks live here so
a laggy/closed browser tab can never leave the motor running unattended.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import can

from ake60_8 import AKE60_8, ERRORS, KD_MAX, KP_MAX, P_MAX, T_MAX, V_MAX

RATE_HZ = 200.0
HISTORY_HZ = 50.0
FEEDBACK_TIMEOUT = 0.3   # s without status frames -> release motor
CLIENT_TIMEOUT = 1.0     # s without UI polling -> release motor (tab closed / network hiccup)
KP_RAMP_S = 0.5          # ramp Kp on enable so a stale setpoint can't cause a jerk
VEL_ACCEL = 40.0         # rad/s^2 slew limit for the velocity setpoint
POS_LIMIT_DEG = math.degrees(P_MAX) - 1.0  # MIT position field only covers +-12.56 rad
# Sine mode: amplitude/frequency glide instead of jumping, so start/stop/retune never jerks the arm.
SINE_AMP_RATE = 30.0     # deg/s change rate of the oscillation amplitude
SINE_FREQ_RATE = 0.5     # Hz/s change rate of the frequency
SINE_PEAK_SPEED = 720.0  # deg/s hard cap on A*2*pi*f (amplitude is reduced to respect it)
# Gait mode = sine + ground-contact detection. Expected current/lag depend on friction, inertia and
# load, so instead of modelling them we learn a per-phase baseline during free cycles and flag
# contact when the measurement departs from it in the "resisting the motion" direction.
GAIT_BINS = 24
GAIT_LEARN_CYCLES = 3
GAIT_MIN_SAMPLES = 3     # per bin before detection is armed
GAIT_DEBOUNCE = 2        # consecutive status frames over threshold -> contact
GAIT_RATE_SLEW = 4.0     # 1/s, how fast the phase-speed factor changes
GAIT_KP_SLEW = 3.0       # 1/s, how fast the stiffness factor changes
STATIC = Path(__file__).parent / "static"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Controller:
    def __init__(self, interface: str, channel: str | None, motor_id: int):
        self.interface, self.channel, self.motor_id = interface, channel, motor_id
        self.lock = threading.Lock()
        self.bus: can.BusABC | None = None
        self.motor: AKE60_8 | None = None
        self.conn_error = ""
        self.enabled = False
        self.mode = "position"          # position | sine | gait | velocity | torque
        self.fault = ""
        # User targets (UI units: deg for position, rad/s, Nm)
        self.target_pos = 0.0
        self.target_vel = 0.0
        self.target_torque = 0.0
        self.kp, self.kd = 10.0, 0.5
        # Velocity mode is a pure Kd loop (torque = Kd * speed error), so it needs more gain to beat friction.
        self.kd_vel = 2.0
        self.max_speed = 90.0           # deg/s slew limit for position setpoint
        # Sine mode user settings
        self.sine_center = 0.0          # deg
        self.sine_amp = 30.0            # deg
        self.sine_freq = 0.5            # Hz
        self.sine_running = False
        # Sine mode internal state (glided toward the user settings)
        self.sine_phase = 0.0           # rad, accumulated so frequency changes keep the motion continuous
        self.sine_amp_cur = 0.0
        self.sine_freq_cur = 0.5
        self.sine_center_cur = 0.0
        self.sine_err2 = 0.0            # EMA of squared tracking error, deg^2
        self.sine_err_peak = 0.0        # decaying peak |error|, deg
        # Gait mode user settings
        self.gait_cur_thresh = 0.4      # A of extra current in the direction of motion
        self.gait_err_thresh = 4.0      # deg of extra lag behind the setpoint
        self.gait_slow = 0.25           # phase-speed factor while in contact
        self.gait_soft = 0.3            # Kp factor while in contact
        self.gait_pen = 3.0             # deg the setpoint may push past the contact point
        self._gait_reset()
        # Internal setpoints actually sent to the motor
        self.sp_pos = 0.0
        self.sp_vel = 0.0
        self.enable_time = 0.0
        self.last_client = time.monotonic()
        self.vel_est = 0.0              # deg/s, estimated from position (erpm needs pole pairs)
        self.history: deque = deque(maxlen=int(HISTORY_HZ * 30))
        self.t0 = time.monotonic()

    # ---------- connection ----------
    def _connect(self):
        channel = self.channel
        if channel is None and self.interface == "slcan":
            ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
            if not ports:
                raise RuntimeError("no USB CAN adapter found")
            channel = ports[0]
        self.bus = can.Bus(interface=self.interface, channel=channel or "can0", bitrate=1_000_000)
        self.motor = AKE60_8(self.bus, self.motor_id)
        self.active_channel = channel
        self.conn_error = ""

    def _disconnect(self, err: str):
        self.enabled = False
        self.conn_error = err
        try:
            if self.bus:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = self.motor = None

    # ---------- commands from UI (called under lock) ----------
    def _release(self, reason: str = ""):
        was = self.enabled
        self.enabled = False
        if reason:
            self.fault = reason
        if self.motor and was:
            for _ in range(3):  # a few copies in case one frame is lost
                self.motor.stop()

    def _reset_setpoints(self):
        st = self.motor.last_status if self.motor else None
        pos = st.pos_deg if st else 0.0
        self.sp_pos = self.target_pos = clamp(pos, -POS_LIMIT_DEG, POS_LIMIT_DEG)
        self.sp_vel = self.target_vel = 0.0
        self.target_torque = 0.0
        self.sine_center = self.sine_center_cur = self.sp_pos
        self.sine_amp_cur = self.sine_phase = 0.0
        self.sine_freq_cur = self.sine_freq
        self.sine_running = False
        self.sine_err2 = self.sine_err_peak = 0.0
        self._gait_reset()
        self.enable_time = time.monotonic()

    def _gait_reset(self):
        """Forget the learned baseline - needed whenever the motion (amp/freq/center) changes."""
        self.gait_state = "learning"    # learning | free | contact | release
        self.gait_bins = [[0.0, 0.0, 0] for _ in range(GAIT_BINS)]  # mean current, mean error, samples
        self.gait_cycles = 0.0          # settled free cycles seen while learning
        self.gait_rate = 1.0            # phase-speed factor (glides toward target)
        self.gait_kp_scale = 1.0        # stiffness factor (glides toward target)
        self.gait_clamp = None          # (limit position deg, direction) while touching ground
        self.gait_hits = 0
        self.gait_contacts = 0
        self.gait_last = None           # info about the last contact, for the UI
        self.gait_res_cur = self.gait_res_err = 0.0
        self.gait_seen_t = -1.0         # last status timestamp processed by the detector
    def enable(self) -> str:
        if not self.motor:
            return "not connected"
        if not self._fresh():
            return "no feedback from motor"
        st = self.motor.last_status
        if self.mode in ("position", "sine", "gait") and abs(st.pos_deg) > POS_LIMIT_DEG:
            return f"position {st.pos_deg:.0f} deg is outside MIT range +-{POS_LIMIT_DEG:.0f} - press Zero first"
        self.fault = ""
        self._reset_setpoints()
        self.enabled = True
        return ""

    def zero(self) -> str:
        if not self.motor:
            return "not connected"
        if self.enabled:
            return "stop the motor before zeroing"
        self.motor.set_origin(permanent=False)
        return ""

    def update(self, d: dict) -> str:
        if "mode" in d and d["mode"] != self.mode:
            if d["mode"] not in ("position", "sine", "gait", "velocity", "torque"):
                return "bad mode"
            self.mode = d["mode"]
            if self.enabled:
                self._reset_setpoints()  # never carry a setpoint across modes
        if "target_pos" in d:
            self.target_pos = clamp(float(d["target_pos"]), -POS_LIMIT_DEG, POS_LIMIT_DEG)
        if "target_vel" in d:
            self.target_vel = clamp(float(d["target_vel"]), -V_MAX, V_MAX)
        if "target_torque" in d:
            self.target_torque = clamp(float(d["target_torque"]), -T_MAX, T_MAX)
        if "kp" in d:
            self.kp = clamp(float(d["kp"]), 0, KP_MAX)
        if "kd" in d:
            self.kd = clamp(float(d["kd"]), 0, KD_MAX)
        if "kd_vel" in d:
            self.kd_vel = clamp(float(d["kd_vel"]), 0.1, KD_MAX)
        if "max_speed" in d:
            self.max_speed = clamp(float(d["max_speed"]), 1, 1000)
        # Baseline is only valid for the motion it was learned on.
        if self.mode == "gait" and any(k in d and float(d[k]) != getattr(self, k)
                                       for k in ("sine_center", "sine_amp", "sine_freq")):
            self._gait_reset()
        for k, lo, hi in (("gait_cur_thresh", 0.05, 10), ("gait_err_thresh", 0.5, 45), ("gait_slow", 0.05, 1),
                          ("gait_soft", 0.05, 1), ("gait_pen", 0, 30)):
            if k in d:
                setattr(self, k, clamp(float(d[k]), lo, hi))
        if "sine_center" in d:
            self.sine_center = clamp(float(d["sine_center"]), -POS_LIMIT_DEG, POS_LIMIT_DEG)
        if "sine_amp" in d:
            self.sine_amp = clamp(float(d["sine_amp"]), 0, 180)
        if "sine_freq" in d:
            self.sine_freq = clamp(float(d["sine_freq"]), 0.05, 5)
        if "sine_running" in d:
            if self.mode not in ("sine", "gait") or not self.enabled:
                return "enable Sine or Gait mode first"
            self.sine_running = bool(d["sine_running"])
            if self.mode == "gait" and self.sine_running:
                self._gait_reset()
        return ""

    def sine_amp_limit(self) -> float:
        """Largest amplitude allowed at the current frequency by the peak-speed cap."""
        return SINE_PEAK_SPEED / (2 * math.pi * self.sine_freq)

    def _fresh(self) -> bool:
        # Check last_status too: time.monotonic() may start near 0 (macOS), so time alone can't tell "never received".
        return (self.motor is not None and self.motor.last_status is not None
                and time.monotonic() - self.motor.last_status_time < FEEDBACK_TIMEOUT)

    # ---------- control loop ----------
    def run(self):
        period = 1.0 / RATE_HZ
        last_hist = 0.0
        last_pos = last_pos_t = None
        next_retry = 0.0
        while True:
            t_start = time.monotonic()
            with self.lock:
                if not self.motor:
                    if t_start >= next_retry:
                        try:
                            self._connect()
                        except Exception as e:
                            self.conn_error = str(e)
                            next_retry = t_start + 2.0
                else:
                    try:
                        self._step(t_start, period)
                        st = self.motor.last_status
                        if st and self.motor.last_status_time != last_pos_t:
                            if last_pos is not None:
                                dt = self.motor.last_status_time - last_pos_t
                                raw = (st.pos_deg - last_pos) / dt if dt > 0 else 0.0
                                self.vel_est += 0.25 * (raw - self.vel_est)  # EMA; 0.1 deg quantization is noisy
                            last_pos, last_pos_t = st.pos_deg, self.motor.last_status_time
                        if st and t_start - last_hist >= 1.0 / HISTORY_HZ:
                            last_hist = t_start
                            sp = self.sp_pos if (self.enabled and self.mode in ("position", "sine", "gait")) else None
                            contact = int(self.enabled and self.mode == "gait" and self.gait_state in ("contact", "release"))
                            self.history.append((round(t_start - self.t0, 3), st.pos_deg, sp,
                                                 st.current_a, round(self.vel_est, 1), contact))
                    except (can.CanError, OSError, ValueError) as e:
                        self._disconnect(f"CAN error: {e}")
                        next_retry = t_start + 2.0
            time.sleep(max(0.0, period - (time.monotonic() - t_start)))

    def _step(self, now: float, dt: float):
        st = self.motor.poll()
        if not self.enabled:
            # Track the real position while idle so the UI slider shows where Enable will hold.
            if st:
                self.target_pos = self.sp_pos = clamp(st.pos_deg, -POS_LIMIT_DEG, POS_LIMIT_DEG)
                self.sine_center = self.sine_center_cur = self.sp_pos
            return
        if not self._fresh():
            return self._release("no feedback from motor")
        if st.error:
            return self._release(f"motor fault: {ERRORS.get(st.error, st.error)}")
        if now - self.last_client > CLIENT_TIMEOUT:
            return self._release("UI disconnected")

        if self.mode == "position":
            step = self.max_speed * dt
            err = self.target_pos - self.sp_pos
            self.sp_pos += clamp(err, -step, step)
            kp = self.kp * min(1.0, (now - self.enable_time) / KP_RAMP_S)
            # While the setpoint is still ramping, feed forward the ramp speed so Kd doesn't brake it.
            ff_v = math.radians(math.copysign(self.max_speed, err)) if abs(err) > step else 0.0
            self.motor.mit(p=math.radians(self.sp_pos), v=ff_v, kp=kp, kd=self.kd)
        elif self.mode == "sine":
            self._step_sine(now, dt, st.pos_deg)
        elif self.mode == "gait":
            self._step_gait(now, dt, st)
        elif self.mode == "velocity":
            step = VEL_ACCEL * dt
            self.sp_vel += clamp(self.target_vel - self.sp_vel, -step, step)
            self.motor.mit(v=self.sp_vel, kd=self.kd_vel)
        else:
            self.motor.mit(t=self.target_torque)

    def _sine_glide(self, dt: float) -> tuple[float, float, float]:
        """Glide frequency, center and amplitude toward the user settings (all rate limited).
        Returns (amp_target, center_error, center_step) for the feed-forward term."""
        df = SINE_FREQ_RATE * dt
        self.sine_freq_cur += clamp(self.sine_freq - self.sine_freq_cur, -df, df)
        amp_target = min(self.sine_amp, self.sine_amp_limit()) if self.sine_running else 0.0
        # Keep the whole swing inside the MIT position range.
        amp_target = min(amp_target, POS_LIMIT_DEG - abs(self.sine_center_cur))
        da = SINE_AMP_RATE * dt
        self.sine_amp_cur += clamp(amp_target - self.sine_amp_cur, -da, da)
        dc = self.max_speed * dt
        c_err = self.sine_center - self.sine_center_cur
        self.sine_center_cur += clamp(c_err, -dc, dc)
        return amp_target, c_err, dc

    def _step_sine(self, now: float, dt: float, pos: float):
        _, c_err, dc = self._sine_glide(dt)
        w = 2 * math.pi * self.sine_freq_cur
        self.sine_phase = (self.sine_phase + w * dt) % (2 * math.pi)
        self.sp_pos = self.sine_center_cur + self.sine_amp_cur * math.sin(self.sine_phase)
        # Velocity feed-forward = derivative of the setpoint; without it Kd drags the arm behind the sine.
        ff_deg_s = self.sine_amp_cur * w * math.cos(self.sine_phase)
        if abs(c_err) > dc:
            ff_deg_s += math.copysign(self.max_speed, c_err)
        kp = self.kp * min(1.0, (now - self.enable_time) / KP_RAMP_S)
        self.motor.mit(p=math.radians(self.sp_pos), v=math.radians(ff_deg_s), kp=kp, kd=self.kd)

        err = pos - self.sp_pos
        self.sine_err2 += 0.01 * (err * err - self.sine_err2)  # ~0.5 s EMA at 200 Hz
        self.sine_err_peak = max(abs(err), self.sine_err_peak * (1 - 0.3 * dt))

    def _step_gait(self, now: float, dt: float, st):
        amp_target, c_err, dc = self._sine_glide(dt)
        touching = self.gait_state == "contact"
        # Slow down and soften while touching the ground; recover smoothly after lift-off.
        r_t = self.gait_slow if touching else 1.0
        self.gait_rate += clamp(r_t - self.gait_rate, -GAIT_RATE_SLEW * dt, GAIT_RATE_SLEW * dt)
        k_t = self.gait_soft if touching else 1.0
        self.gait_kp_scale += clamp(k_t - self.gait_kp_scale, -GAIT_KP_SLEW * dt, GAIT_KP_SLEW * dt)

        w = 2 * math.pi * self.sine_freq_cur * self.gait_rate
        self.sine_phase = (self.sine_phase + w * dt) % (2 * math.pi)
        raw = self.sine_center_cur + self.sine_amp_cur * math.sin(self.sine_phase)
        raw_v = self.sine_amp_cur * w * math.cos(self.sine_phase)
        if abs(c_err) > dc:
            raw_v += math.copysign(self.max_speed, c_err)
        d_raw = (raw_v > 0) - (raw_v < 0)

        # Don't let the setpoint dig further than gait_pen past the contact point. min/max of two
        # continuous signals is continuous, so entering/leaving the clamp never jumps.
        sp, ff = raw, raw_v
        if self.gait_clamp:
            lim, cdir = self.gait_clamp
            if cdir * (raw - lim) > 0:
                sp, ff = lim, 0.0
            elif self.gait_state == "release":
                self.gait_clamp = None
                self.gait_state = "free"
        if self.gait_state == "contact" and d_raw == -self.gait_clamp[1]:
            self.gait_state = "release"  # sine turned around: foot lifts off

        kp = self.kp * min(1.0, (now - self.enable_time) / KP_RAMP_S) * self.gait_kp_scale
        self.sp_pos = sp
        self.motor.mit(p=math.radians(sp), v=math.radians(ff), kp=kp, kd=self.kd)

        # Detection runs once per status frame (motor uploads at its CAN feedback rate, ~50 Hz here).
        if self.motor.last_status_time == self.gait_seen_t:
            return
        frame_dt = min(self.motor.last_status_time - self.gait_seen_t, 0.1) if self.gait_seen_t >= 0 else 0.0
        self.gait_seen_t = self.motor.last_status_time
        err = st.pos_deg - sp
        self.sine_err2 += 0.05 * (err * err - self.sine_err2)
        self.sine_err_peak = max(abs(err), self.sine_err_peak * (1 - 0.3 * dt))
        b = self.gait_bins[int(self.sine_phase / (2 * math.pi) * GAIT_BINS) % GAIT_BINS]
        settled = (self.sine_running and abs(self.sine_amp_cur - amp_target) < 0.5
                   and abs(self.sine_freq_cur - self.sine_freq) < 0.01 and self.sine_amp_cur > 1.0)

        if self.gait_state == "learning":
            if settled:
                b[2] += 1
                b[0] += (st.current_a - b[0]) / b[2]
                b[1] += (err - b[1]) / b[2]
                self.gait_cycles += w / (2 * math.pi) * frame_dt
                if (self.gait_cycles >= GAIT_LEARN_CYCLES
                        and all(x[2] >= GAIT_MIN_SAMPLES for x in self.gait_bins)):
                    self.gait_state = "free"
            return
        if self.gait_state != "free" or not settled:
            return
        self.gait_res_cur = st.current_a - b[0]
        self.gait_res_err = err - b[1]
        # Resistance pushes current up in the direction of motion and makes the arm lag behind.
        hit = d_raw != 0 and (d_raw * self.gait_res_cur > self.gait_cur_thresh
                              or -d_raw * self.gait_res_err > self.gait_err_thresh)
        self.gait_hits = self.gait_hits + 1 if hit else 0
        if self.gait_hits >= GAIT_DEBOUNCE:
            self.gait_state = "contact"
            self.gait_hits = 0
            self.gait_clamp = (st.pos_deg + d_raw * self.gait_pen, d_raw)
            self.gait_contacts += 1
            self.gait_last = {"pos": round(st.pos_deg, 1), "res_cur": round(self.gait_res_cur, 2),
                              "res_err": round(self.gait_res_err, 1), "t": round(now - self.t0, 2)}
        elif not hit:
            # Track slow drift (warm-up, load change) - only with clean free-motion samples.
            b[0] += 0.05 * (st.current_a - b[0])
            b[1] += 0.05 * (err - b[1])

    def state(self, since: float) -> dict:
        self.last_client = time.monotonic()
        st = self.motor.last_status if self.motor else None
        return {
            "connected": self.motor is not None,
            "conn_error": self.conn_error,
            "channel": getattr(self, "active_channel", None),
            "motor_id": self.motor_id,
            "fresh": self._fresh(),
            "enabled": self.enabled,
            "mode": self.mode,
            "fault": self.fault,
            "status": None if st is None else {
                "pos": st.pos_deg, "vel": round(self.vel_est, 1), "erpm": st.speed_erpm,
                "current": st.current_a, "temp": st.temp_c, "error": ERRORS.get(st.error, st.error),
            },
            "target": {
                "pos": self.target_pos, "vel": self.target_vel, "torque": self.target_torque,
                "kp": self.kp, "kd": self.kd, "kd_vel": self.kd_vel, "max_speed": self.max_speed, "sp_pos": self.sp_pos,
            },
            "sine": {
                "center": self.sine_center, "amp": self.sine_amp, "freq": self.sine_freq,
                "running": self.sine_running, "amp_cur": round(self.sine_amp_cur, 1),
                "freq_cur": round(self.sine_freq_cur, 2), "amp_limit": round(self.sine_amp_limit(), 1),
                "peak_speed": round(self.sine_amp_cur * 2 * math.pi * self.sine_freq_cur, 0),
                "err_rms": round(math.sqrt(self.sine_err2), 2), "err_peak": round(self.sine_err_peak, 2),
            },
            "gait": {
                "state": self.gait_state, "cycles": round(self.gait_cycles, 1), "learn_cycles": GAIT_LEARN_CYCLES,
                "contacts": self.gait_contacts, "last": self.gait_last,
                "res_cur": round(self.gait_res_cur, 2), "res_err": round(self.gait_res_err, 1),
                "rate": round(self.gait_rate, 2), "kp_scale": round(self.gait_kp_scale, 2),
                "cur_thresh": self.gait_cur_thresh, "err_thresh": self.gait_err_thresh,
                "slow": self.gait_slow, "soft": self.gait_soft, "pen": self.gait_pen,
            },
            "limits": {"pos": POS_LIMIT_DEG, "vel": V_MAX, "torque": T_MAX, "kp": KP_MAX, "kd": KD_MAX},
            "history": [h for h in self.history if h[0] > since],
        }


def make_handler(ctl: Controller):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # keep console quiet at 20 Hz polling
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = (STATIC / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith("/api/state"):
                since = float(self.path.partition("since=")[2] or -1)
                with ctl.lock:
                    self._json(ctl.state(since))
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            data = json.loads(self.rfile.read(n) or b"{}")
            with ctl.lock:
                if self.path == "/api/set":
                    err = ctl.update(data)
                elif self.path == "/api/enable":
                    err = ctl.enable()
                elif self.path == "/api/stop":
                    ctl._release()
                    ctl.fault = ""
                    err = ""
                elif self.path == "/api/zero":
                    err = ctl.zero()
                else:
                    return self._json({"error": "not found"}, 404)
            self._json({"ok": not err, "error": err}, 200 if not err else 400)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="AKE60-8 web control UI")
    ap.add_argument("--interface", default="slcan")
    ap.add_argument("--channel", default=None)
    ap.add_argument("--id", type=int, default=101)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8060)
    args = ap.parse_args()

    ctl = Controller(args.interface, args.channel, args.id)
    threading.Thread(target=ctl.run, daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(ctl))
    print(f"AKE60-8 UI on http://{args.host}:{args.port}  (Ctrl+C to quit)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with ctl.lock:
            ctl._release()
            if ctl.bus:
                ctl.bus.shutdown()
        print("motor released, bye")


if __name__ == "__main__":
    main()
