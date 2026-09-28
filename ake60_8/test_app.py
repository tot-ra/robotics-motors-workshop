"""Safety logic of the UI controller, tested on a python-can virtual bus (no hardware)."""

import time

import can
import pytest

from app import CLIENT_TIMEOUT, Controller


@pytest.fixture
def rig():
    ctl = Controller("virtual", "ui-test", motor_id=101)
    ctl._connect()
    fake = can.Bus(interface="virtual", channel="ui-test")

    def status(pos_deg=10.0, err=0):
        raw = int(pos_deg * 10)
        fake.send(can.Message(arbitration_id=0x2900 | 101, is_extended_id=True,
                              data=raw.to_bytes(2, "big", signed=True) + bytes([0, 0, 0, 0, 25, err])))
        time.sleep(0.01)
        ctl._step(time.monotonic(), 0.005)

    yield ctl, fake, status
    fake.shutdown()
    ctl.bus.shutdown()


def drain(bus):
    msgs = []
    while (m := bus.recv(0.01)) is not None:
        msgs.append(m)
    return msgs


def test_enable_holds_current_position(rig):
    ctl, fake, status = rig
    status(10.0)
    assert ctl.enable() == ""
    assert ctl.target_pos == pytest.approx(10.0)


def test_enable_refused_without_feedback(rig):
    ctl, _, _ = rig
    assert "feedback" in ctl.enable()


def test_watchdog_releases_when_ui_stops_polling(rig):
    ctl, fake, status = rig
    status()
    ctl.last_client = time.monotonic()
    ctl.enable()
    ctl.last_client = time.monotonic() - CLIENT_TIMEOUT - 0.1
    status()
    assert not ctl.enabled and ctl.fault == "UI disconnected"
    # The last frame sent must be the all-zero "release" command
    last = [m for m in drain(fake) if m.arbitration_id == (8 << 8) | 101][-1]
    assert last.data[0:3] == bytes(3)  # kp = kd = 0


def test_motor_fault_releases(rig):
    ctl, _, status = rig
    status()
    ctl.last_client = time.monotonic()
    ctl.enable()
    status(err=2)
    assert not ctl.enabled and "over-current" in ctl.fault


def test_mode_switch_resets_setpoints(rig):
    ctl, _, status = rig
    status(20.0)
    ctl.last_client = time.monotonic()
    ctl.enable()
    ctl.update({"mode": "velocity", "target_vel": 5})
    ctl.update({"mode": "position"})
    assert ctl.target_vel == 0 and ctl.target_pos == pytest.approx(20.0)


class StubMotor:
    """Records MIT commands; enough for testing the sine trajectory generator."""
    def __init__(self):
        self.cmds = []

    def mit(self, **kw):
        self.cmds.append(kw)


def run_sine(ctl, seconds, dt=0.005):
    import math
    out = []
    for _ in range(int(seconds / dt)):
        ctl._step_sine(time.monotonic(), dt, ctl.sp_pos)
        out.append((ctl.sp_pos, math.degrees(ctl.motor.cmds[-1]["v"])))
    return out


@pytest.fixture
def sine_ctl():
    ctl = Controller("virtual", "unused", 101)
    ctl.motor = StubMotor()
    ctl.mode, ctl.enabled = "sine", True
    ctl.sine_center = ctl.sine_center_cur = 10.0
    ctl.sine_freq = ctl.sine_freq_cur = 0.5
    ctl.enable_time = time.monotonic() - 10
    return ctl


def test_sine_starts_smoothly_and_is_continuous(sine_ctl):
    ctl = sine_ctl
    ctl.sine_amp, ctl.sine_running = 30.0, True
    traj = run_sine(ctl, 4.0)
    # amplitude ramps at SINE_AMP_RATE: first 0.2 s stays within a few degrees of center
    assert all(abs(p - 10.0) < 6.5 for p, _ in traj[:40])
    # after ramp-up the full swing is reached
    assert max(p for p, _ in traj) == pytest.approx(40.0, abs=0.5)
    # no jumps: max step per 5 ms tick bounded by peak speed (30*2*pi*0.5 = 94 deg/s)
    assert max(abs(b[0] - a[0]) for a, b in zip(traj, traj[1:])) < 94 * 0.005 * 1.1


def test_sine_velocity_feedforward_matches_derivative(sine_ctl):
    ctl = sine_ctl
    ctl.sine_amp = ctl.sine_amp_cur = 30.0
    ctl.sine_running = True
    traj = run_sine(ctl, 1.0)
    for (p0, _), (p1, v) in zip(traj[100:150], traj[101:151]):
        assert v == pytest.approx((p1 - p0) / 0.005, abs=3.0)


def test_sine_peak_speed_cap(sine_ctl):
    from app import SINE_PEAK_SPEED
    ctl = sine_ctl
    ctl.sine_amp, ctl.sine_freq, ctl.sine_freq_cur, ctl.sine_running = 120.0, 3.0, 3.0, True
    run_sine(ctl, 6.0)
    assert ctl.sine_amp_cur * 2 * 3.14159 * 3.0 <= SINE_PEAK_SPEED + 1


def test_sine_stop_glides_back_to_center(sine_ctl):
    ctl = sine_ctl
    ctl.sine_amp = ctl.sine_amp_cur = 30.0
    ctl.sine_running = True
    run_sine(ctl, 0.3)
    ctl.sine_running = False
    run_sine(ctl, 1.5)
    assert ctl.sine_amp_cur == 0.0 and ctl.sp_pos == pytest.approx(10.0)


def test_sine_start_requires_enabled_sine_mode(rig):
    ctl, _, _ = rig
    assert ctl.update({"sine_running": True}) != ""


class SimJoint:
    """1-DOF joint driven by the MIT law, with an optional 'ground' (stiff spring) - for gait tests.
    Status frames are published at 50 Hz like the real motor; the controller runs at 200 Hz."""
    J, B, KT = 0.002, 0.03, 0.8        # kg*m^2, Nm/(rad/s), Nm/A
    WALL_K = 60.0                      # Nm/rad ground stiffness

    def __init__(self, pos_deg=10.0):
        import math
        self.math = math
        self.p, self.v = math.radians(pos_deg), 0.0
        self.cmd = dict(p=self.p, v=0.0, kp=0.0, kd=0.0, t=0.0)
        self.wall = None               # ground position in deg (blocks motion above it)
        self.t = 0.0
        self.last_status = None
        self.last_status_time = -1.0
        self.max_wall_torque = 0.0

    def mit(self, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0):
        self.cmd = dict(p=p, v=v, kp=kp, kd=kd, t=t)

    def advance(self, dt, publish):
        from ake60_8 import Status
        c = self.cmd
        for _ in range(20):  # substeps: the ground spring is stiff
            tau = c["kp"] * (c["p"] - self.p) + c["kd"] * (c["v"] - self.v) + c["t"]
            wall_tau = 0.0
            if self.wall is not None and self.p > self.math.radians(self.wall):
                wall_tau = -self.WALL_K * (self.p - self.math.radians(self.wall)) - 0.05 * self.v
                self.max_wall_torque = max(self.max_wall_torque, -wall_tau)
            a = (tau + wall_tau - self.B * self.v) / self.J
            self.v += a * dt / 20
            self.p += self.v * dt / 20
        self.t += dt
        if publish:
            self.last_status = Status(round(self.math.degrees(self.p), 1), 0.0, round(tau / self.KT, 2), 25, 0)
            self.last_status_time = self.t


def run_gait(ctl, seconds, dt=0.005):
    m = ctl.motor
    for i in range(int(seconds / dt)):
        m.advance(dt, publish=(i % 4 == 0))
        ctl._step_gait(m.t, dt, m.last_status)


@pytest.fixture
def gait_ctl():
    ctl = Controller("virtual", "unused", 101)
    ctl.motor = SimJoint(10.0)
    ctl.motor.advance(0.005, True)
    ctl.mode, ctl.enabled = "gait", True
    ctl.kp, ctl.kd = 20.0, 0.5
    ctl.sine_center = ctl.sine_center_cur = 10.0
    ctl.sine_amp, ctl.sine_freq = 30.0, 0.5
    ctl.sine_freq_cur = 0.5
    ctl.enable_time = -10
    ctl.sine_running = True
    return ctl


def test_gait_learns_then_no_false_contacts(gait_ctl):
    ctl = gait_ctl
    run_gait(ctl, 9.0)                  # 1 s ramp-up + 3 learning cycles (6 s) + margin
    assert ctl.gait_state == "free"
    run_gait(ctl, 10.0)                 # 5 more free cycles
    assert ctl.gait_contacts == 0


def test_gait_detects_ground_slows_and_limits_force(gait_ctl):
    ctl = gait_ctl
    run_gait(ctl, 9.0)
    assert ctl.gait_state == "free"
    ctl.motor.wall = 25.0               # ground 15 deg before the top of the 10+-30 swing
    saw_slow = False
    for _ in range(40):                 # 10 s in 0.25 s chunks
        run_gait(ctl, 0.25)
        saw_slow |= ctl.gait_rate < 0.5
    assert ctl.gait_contacts >= 3       # detected on every step
    assert saw_slow
    assert ctl.gait_last["pos"] == pytest.approx(25.0, abs=4.0)
    # Setpoint clamp + softened Kp keep the push into the ground small
    # (without the clamp Kp=20 would press 15 deg * 20 Nm/rad = 5.2 Nm).
    assert ctl.motor.max_wall_torque < 2.0


def test_gait_motion_change_relearns(gait_ctl):
    ctl = gait_ctl
    run_gait(ctl, 9.0)
    assert ctl.gait_state == "free"
    ctl.update({"sine_amp": 20})
    assert ctl.gait_state == "learning"
