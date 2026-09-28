"""Minimal CAN controller for the CubeMars AKE60-8 KV80 actuator (MIT / force-control mode).

Protocol source: CubeMars "AK Series Module Product Manual" Ver.3.0.1 (2025-03-14), section 4.2/4.3.
  - CAN 1 Mbps, extended frames.
  - Command frame ID = (8 << 8) | motor_id  (8 = MIT control mode ID)
  - Motor uploads status periodically (1-500 Hz, configured in the CubeMars upper computer).

Default adapter: CANine / CANable with slcan firmware on USB (auto-detects /dev/cu.usbmodem*).
Linux SocketCAN instead: --interface socketcan --channel can0

Usage examples:
    python ake60_8.py --id 1 monitor
    python ake60_8.py --id 1 pos 1.57 --kp 20 --kd 1
    python ake60_8.py --id 1 vel 3.0 --kd 1
    python ake60_8.py --id 1 torque 0.5
    python ake60_8.py --id 1 zero
"""

from __future__ import annotations  # allows `X | None` hints on Python 3.9

import argparse
import glob
import struct
import time
from dataclasses import dataclass

import can

# AKE60-8 MIT-mode ranges (manual v3.0.1, "Parameter Ranges" table). Output-shaft units.
P_MIN, P_MAX = -12.56, 12.56   # rad
V_MIN, V_MAX = -40.0, 40.0     # rad/s
T_MIN, T_MAX = -15.0, 15.0     # Nm
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

MODE_MIT = 8
MODE_SET_ORIGIN = 5

ERRORS = {
    0: "ok", 1: "motor over-temperature", 2: "over-current", 3: "over-voltage",
    4: "under-voltage", 5: "encoder fault", 6: "MOSFET over-temperature", 7: "motor locked",
}


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    # Use (2^bits - 1) instead of the manual's 2^bits, otherwise x_max overflows the bit field.
    return round((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def pack_mit(p: float, v: float, kp: float, kd: float, t: float) -> bytes:
    """Pack an MIT command. Note: AK V3/AKE layout is Kp, Kd, P, V, T (differs from old AK V1/V2 firmware)."""
    p_i = float_to_uint(p, P_MIN, P_MAX, 16)
    v_i = float_to_uint(v, V_MIN, V_MAX, 12)
    kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_i = float_to_uint(t, T_MIN, T_MAX, 12)
    return bytes([
        kp_i >> 4,
        ((kp_i & 0xF) << 4) | (kd_i >> 8),
        kd_i & 0xFF,
        p_i >> 8,
        p_i & 0xFF,
        v_i >> 4,
        ((v_i & 0xF) << 4) | (t_i >> 8),
        t_i & 0xFF,
    ])


@dataclass
class Status:
    pos_deg: float    # output position, degrees
    speed_erpm: float # electrical RPM (divide by pole pairs * gear ratio for shaft RPM)
    current_a: float
    temp_c: int
    error: int

    def __str__(self):
        return (f"pos={self.pos_deg:8.1f} deg  speed={self.speed_erpm:8.0f} erpm  "
                f"I={self.current_a:6.2f} A  T={self.temp_c:3d} C  err={ERRORS.get(self.error, self.error)}")


def unpack_status(data: bytes) -> Status:
    pos, spd, cur, temp, err = struct.unpack(">hhhbB", bytes(data[:8]))
    return Status(pos * 0.1, spd * 10.0, cur * 0.01, temp, err)


class AKE60_8:
    def __init__(self, bus: can.BusABC, motor_id: int = 1):
        self.bus = bus
        self.id = motor_id
        self.last_status: Status | None = None
        self.last_status_time = 0.0  # time.monotonic() of last status frame, for staleness checks

    def _send(self, mode: int, data: bytes):
        self.bus.send(can.Message(arbitration_id=(mode << 8) | self.id, data=data, is_extended_id=True))

    def mit(self, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0):
        self._send(MODE_MIT, pack_mit(p, v, kp, kd, t))

    def stop(self):
        """Zero all gains and torque - motor goes limp."""
        self.mit(0, 0, 0, 0, 0)

    def set_origin(self, permanent: bool = False):
        self._send(MODE_SET_ORIGIN, bytes([1 if permanent else 0]))

    def poll(self, timeout: float = 0.0) -> Status | None:
        """Drain RX queue and keep the latest status frame from our motor."""
        msg = self.bus.recv(timeout)
        while msg is not None:
            # Status frames carry the driver ID in the low byte; skip our own MIT echoes.
            if (msg.is_extended_id and (msg.arbitration_id & 0xFF) == self.id
                    and (msg.arbitration_id >> 8) != MODE_MIT and len(msg.data) == 8):
                self.last_status = unpack_status(msg.data)
                self.last_status_time = time.monotonic()
            msg = self.bus.recv(0.0)
        return self.last_status


def run_loop(motor: AKE60_8, cmd: dict, duration: float, rate_hz: float = 200.0):
    """MIT mode expects a steady command stream, so resend the setpoint at a fixed rate."""
    period = 1.0 / rate_hz
    t_end = time.monotonic() + duration if duration > 0 else float("inf")
    next_print = 0.0
    try:
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            if cmd:
                motor.mit(**cmd)
            st = motor.poll()
            if st and t0 >= next_print:
                print(st)
                next_print = t0 + 0.1
                if st.error:
                    print("Motor fault - stopping")
                    break
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        # Monitor mode must stay passive - don't switch the motor into MIT mode.
        if cmd:
            motor.stop()
            print("stopped")


def main():
    ap = argparse.ArgumentParser(description="CubeMars AKE60-8 MIT-mode control")
    ap.add_argument("--interface", default="slcan", help="python-can interface: slcan (CANine/CANable), socketcan, gs_usb...")
    ap.add_argument("--channel", default=None, help="default: first /dev/cu.usbmodem* (slcan); e.g. can0 for socketcan")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--id", type=int, default=101, help="motor CAN ID (workshop motor is 101 = 0x65)")
    ap.add_argument("--duration", type=float, default=0, help="seconds to run, 0 = until Ctrl+C")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("monitor", help="print status only, no commands")
    p = sub.add_parser("pos", help="hold position (rad)")
    p.add_argument("value", type=float)
    p.add_argument("--kp", type=float, default=10.0)
    p.add_argument("--kd", type=float, default=0.5)
    v = sub.add_parser("vel", help="run at velocity (rad/s)")
    v.add_argument("value", type=float)
    v.add_argument("--kd", type=float, default=1.0)
    t = sub.add_parser("torque", help="apply torque (Nm)")
    t.add_argument("value", type=float)
    z = sub.add_parser("zero", help="set current position as origin")
    z.add_argument("--permanent", action="store_true", help="save origin to flash")
    args = ap.parse_args()

    if args.channel is None:
        if args.interface == "slcan":
            ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
            if not ports:
                ap.error("no USB CAN adapter found, pass --channel")
            args.channel = ports[0]
        else:
            args.channel = "can0"
    print(f"using {args.interface} on {args.channel} @ {args.bitrate} bps")

    with can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate) as bus:
        motor = AKE60_8(bus, args.id)
        if args.cmd == "zero":
            motor.set_origin(args.permanent)
            print("origin set")
            return
        cmd = {
            "monitor": {},
            "pos": lambda: dict(p=args.value, kp=args.kp, kd=args.kd),
            "vel": lambda: dict(v=args.value, kd=args.kd),
            "torque": lambda: dict(t=args.value),
        }[args.cmd]
        run_loop(motor, cmd() if callable(cmd) else cmd, args.duration)


if __name__ == "__main__":
    main()
